# CentoroidalManager の分析結果

## 1. update()
### 1.1 MPC
```cpp
  // Set MPC state
  if(config().useActualStateForMpc)
  {
    // ゼロモーメントの代入
    mpcCom_ = actualCom();  
    mpcComVel_ = ctl().realRobot().comVelocity();
  }
  else
  {
    // Task targets are the planned state in the previous step
    mpcCom_ = ctl().comTask_->com();
    mpcComVel_ = ctl().comTask_->refVel();
  }
  refZmp_ = ctl().footManager_->calcRefZmp(ctl().t());
```
実機を使うか場合と使わない場合とで処理を分け、mpcで用いるcomの座標と速度を初期化

```cpp
// Run MPC
  runMpc();
```
MPC実行
#### 1.1.1 MPCとは何か
MPC（Model Predictive Control、モデル予測制御）とは、未来の挙動を予測して最適な操作を決定する制御手法である。

---

### 1.2 測定ZMPの計算
```cpp
  // Calculate measured ZMP
  {
    std::unordered_map<Foot, sva::ForceVecd> sensorWrenchList;
    for(const auto & foot : ctl().footManager_->getCurrentContactFeet())
    {
      const auto & surfaceName = ctl().footManager_->surfaceName(foot);
      const auto & sensorName = ctl().robot().indirectSurfaceForceSensor(surfaceName).name();
      const auto & sensor = ctl().robot().forceSensor(sensorName);
      const auto & sensorWrench = sensor.worldWrenchWithoutGravity(ctl().robot());
      sensorWrenchList.emplace(foot, sensorWrench);
    }
    measuredZMP_ = calcZmp(sensorWrenchList, refZmp_.z());
  }
```
センサー名称とデータをsensorWrenchListに格納し、それをもとにZMPを計算。

#### calcZmp()
```cpp
Eigen::Vector3d CentroidalManager::calcZmp(const std::unordered_map<Foot, sva::ForceVecd> & wrenchList,
                                           double zmpPlaneHeight,
                                           const Eigen::Vector3d & zmpPlaneNormal) const
{
  sva::ForceVecd totalWrench = sva::ForceVecd::Zero();

  // モーメントの総和
  for(const auto & wrenchKV : wrenchList)
  {
    totalWrench += wrenchKV.second;
  }

  Eigen::Vector3d zmpPlaneOrigin = Eigen::Vector3d(0, 0, zmpPlaneHeight);
  Eigen::Vector3d zmp = zmpPlaneOrigin;

  if(totalWrench.force().z() > 0)
  {
    Eigen::Vector3d momentInZmpPlane = totalWrench.moment() - zmpPlaneOrigin.cross(totalWrench.force());
    zmp += zmpPlaneNormal.cross(momentInZmpPlane) / totalWrench.force().z();
  }

  return zmp;
}
```

---

### 1.3 関節角度の計算
```cpp
  // Calculate target wrench
  {
    controlZmp_ = plannedZmp_;
    controlForceZ_ = plannedForceZ_;

    // Compensate ZMP delay
    // See equation (10) of https://ieeexplore.ieee.org/abstract/document/6094838
    Eigen::Vector3d refZmpVel = ctl().footManager_->calcRefZmp(ctl().t(), 1);
    controlZmp_.head<2>() += config().zmpVelGain * refZmpVel.head<2>();

    // Apply DCM feedback
    if(config().enableZmpFeedback)
    {
      double omega = std::sqrt(plannedForceZ_ / (robotMass_ * (mpcCom_.z() - refZmp_.z())));
      Eigen::Vector3d plannedDcm = ctl().comTask_->com() + ctl().comTask_->refVel() / omega;
      Eigen::Vector3d actualDcm = actualCom() + ctl().realRobot().comVelocity() / omega;
      Eigen::Vector3d dcmError = actualDcm - plannedDcm;

      // Estimate and compensate for DCM bias
      if(config().dcmEstimatorConfig.enableDcmEstimator)
      {
        if(omega != dcmEstimator_->getLipmNaturalFrequency())
        {
          dcmEstimator_->setLipmNaturalFrequency(omega);
        }

        const Eigen::Matrix3d & baseOrientation = ctl().robot().posW().rotation().transpose();
        if(requireDcmEstimatorReset_)
        {
          dcmEstimator_->resetWithMeasurements(actualDcm.head<2>(), measuredZMP_.head<2>(), baseOrientation, true);
          requireDcmEstimatorReset_ = false;
        }
        else
        {
          dcmEstimator_->setInputs(actualDcm.head<2>(), measuredZMP_.head<2>(), baseOrientation);
        }

        dcmEstimator_->update();

        if(config().dcmEstimatorConfig.dcmCorrectionMode == "Bias")
        {
          dcmError.head<2>() -= dcmEstimator_->getBias();
        }
        else if(config().dcmEstimatorConfig.dcmCorrectionMode == "Filter")
        {
          dcmError.head<2>() = dcmEstimator_->getUnbiasedDCM();
        }
        // else if(config().dcmEstimatorConfig.dcmCorrectionMode == "None")
        // {
        // }
      }
      else
      {
        requireDcmEstimatorReset_ = true;
        dcmEstimator_->setBias(Eigen::Vector2d::Zero());
      }

      controlZmp_.head<2>() += config().dcmGainP * dcmError.head<2>();
    }

    // Apply ForceZ feedback
    if(config().enableComZFeedback)
    {
      double plannedComZ = ctl().comTask_->com().z();
      double actualComZ = actualCom().z();
      double plannedComVelZ = ctl().comTask_->refVel().z();
      double actualComVelZ = ctl().realRobot().comVelocity().z();
      controlForceZ_ -=
          config().comZGainP * (actualComZ - plannedComZ) + config().comZGainD * (actualComVelZ - plannedComVelZ);
    }

    // Convert ZMP to wrench and distribute
    contactList_ = ctl().footManager_->calcCurrentContactList();
    wrenchDist_ = std::make_shared<ForceColl::WrenchDistribution>(ForceColl::getContactVecFromMap(contactList_),
                                                                  config().wrenchDistConfig);
    Eigen::Vector3d comForWrenchDist =
        (config().useActualComForWrenchDist ? actualComUnbiased() : ctl().comTask_->com());
    sva::ForceVecd controlWrench;
    controlWrench.force() << controlForceZ_ / (comForWrenchDist.z() - refZmp_.z())
                                 * (comForWrenchDist.head<2>() - controlZmp_.head<2>()),
        controlForceZ_;
    controlWrench.moment().setZero(); // Moment is represented around CoM
    wrenchDist_->run(controlWrench, comForWrenchDist);
  }
  ```

#### 1.3.1 ZMPの初期化と速度補償
```cpp
controlZmp_ = plannedZmp_;
controlForceZ_ = plannedForceZ_;

Eigen::Vector3d refZmpVel = ctl().footManager_->calcRefZmp(ctl().t(), 1);
controlZmp_.head<2>() += config().zmpVelGain * refZmpVel.head<2>();
```
ZMP速度補償（遅延補償）により、リアルタイム制御の遅れを補う。
[参考](https://ieeexplore.ieee.org/document/6094838)








