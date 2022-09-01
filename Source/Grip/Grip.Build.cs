/**
*
* GRIP build.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright 2015-2017 Caged Element Inc.
*
***********************************************************************************/

#define GRIP_USE_STEAM

using UnrealBuildTool;

public class Grip : ModuleRules
{
	public Grip(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[] {
				"Core",
				"CoreUObject",
				"Engine",
				"EngineSettings",
				"UMG",
				"PhysX",
				"Networking",
				"ProceduralMeshComponent",
				"RenderCore",
				"RHI",
				"MediaAssets",
				"Http",
				"Json",
				"JsonUtilities"
			}
			);

		PrivateDependencyModuleNames.AddRange(
			new string[] {
				"InputCore",
				"Slate",
				"SlateCore",
				"Sockets",
				"OnlineSubsystem",
				"OnlineSubsystemUtils"
			}
		);

#if GRIP_USE_STEAM
		if ((Target.Platform == UnrealTargetPlatform.Win64) || (Target.Platform == UnrealTargetPlatform.Win32) || (Target.Platform == UnrealTargetPlatform.Linux) || (Target.Platform == UnrealTargetPlatform.Mac))
		{
			PublicDependencyModuleNames.AddRange(new string[] { "Steamworks" });
		}

		if ((Target.Platform == UnrealTargetPlatform.Win32) || (Target.Platform == UnrealTargetPlatform.Win64) || (Target.Platform == UnrealTargetPlatform.Linux) || (Target.Platform == UnrealTargetPlatform.Mac))
		{
			DynamicallyLoadedModuleNames.Add("OnlineSubsystemSteam");
		}
#endif // GRIP_USE_STEAM

	}
}
