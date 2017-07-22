/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/controller/StateSpaceControllerCoeffs.h>
#include <frc/controller/StateSpaceLoop.h>
#include <frc/controller/StateSpaceObserverCoeffs.h>
#include <frc/controller/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<4, 2, 2> MakeDrivetrainPlantCoeffs();
frc::StateSpaceControllerCoeffs<4, 2, 2> MakeDrivetrainControllerCoeffs();
frc::StateSpaceObserverCoeffs<4, 2, 2> MakeDrivetrainObserverCoeffs();
frc::StateSpaceLoop<4, 2, 2> MakeDrivetrainLoop();
