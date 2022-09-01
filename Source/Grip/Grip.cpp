/**
*
* The GRIP module.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
***********************************************************************************/

#include "Grip.h"
#include "Modules/ModuleManager.h"
#include "System/GameConfiguration.h"

IMPLEMENT_PRIMARY_GAME_MODULE( FDefaultGameModuleImpl, Grip, "Grip" );

DEFINE_LOG_CATEGORY(GripLog);
DEFINE_LOG_CATEGORY(GripAILog);
DEFINE_LOG_CATEGORY(GripTeleportationLog);
