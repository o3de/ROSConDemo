/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ApplePicker/ApplePickerComponent.h"
#include "ApplePicker/GatheringRowComponent.h"
#include "ApplePicker/KrakenEffectorComponent.h"
#include "DemoStatistics/DemoStatisticsComponent.h"
#include "FruitStorage/FruitStorageComponent.h"
#include "Manipulator/KrakenManipulatorController.h"
#include "KrakenCamera/FollowingCameraComponent.h"
#include "RobotHarvestingSampleSystemComponent.h"
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

namespace RobotHarvestingSample
{
    class RobotHarvestingSampleModule : public AZ::Module
    {
    public:
        AZ_RTTI(RobotHarvestingSampleModule, "{E38575E4-7D2F-4617-B938-416E8C1C07B4}", AZ::Module);
        AZ_CLASS_ALLOCATOR(RobotHarvestingSampleModule, AZ::SystemAllocator, 0);

        RobotHarvestingSampleModule()
            : AZ::Module()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                { RobotHarvestingSampleSystemComponent::CreateDescriptor(),
                  AppleKraken::ApplePickerComponent::CreateDescriptor(),
                  AppleKraken::GatheringRowComponent::CreateDescriptor(),
                  AppleKraken::KrakenEffectorComponent::CreateDescriptor(),
                  AppleKraken::FruitStorageComponent::CreateDescriptor(),
                  AppleKraken::DemoStatisticsComponent::CreateDescriptor(),
                  AppleKraken::ManipulatorController::CreateDescriptor(),
                  AppleKraken::FollowingCameraComponent::CreateDescriptor() });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<RobotHarvestingSampleSystemComponent>(),
            };
        }
    };
} // namespace RobotHarvestingSample

AZ_DECLARE_MODULE_CLASS(Gem_RobotHarvestingSample, RobotHarvestingSample::RobotHarvestingSampleModule)
