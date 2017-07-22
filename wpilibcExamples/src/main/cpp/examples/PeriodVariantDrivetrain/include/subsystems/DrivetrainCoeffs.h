/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/controller/PeriodVariantKalmanFilterCoeffs.h>
#include <frc/controller/PeriodVariantLoop.h>
#include <frc/controller/PeriodVariantPlantCoeffs.h>
#include <frc/controller/StateSpaceControllerCoeffs.h>

frc::PeriodVariantPlantCoeffs<4, 2, 2> MakeDrivetrainPlantCoeffs();
frc::StateSpaceControllerCoeffs<4, 2, 2> MakeDrivetrainControllerCoeffs();
frc::PeriodVariantKalmanFilterCoeffs<4, 2, 2> MakeDrivetrainObserverCoeffs();
frc::PeriodVariantLoop<4, 2, 2> MakeDrivetrainLoop();
