/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ApplePicker/ApplePickerComponent.h"
#include "ApplePicker/GatheringRowComponent.h"
#include "ApplePicker/KrakenEffectorComponent.h"
#include "DemoStatistics/DemoStatisticsComponent.h"
#include "FruitStorage/FruitStorageComponent.h"
#include "Manipulator/KrakenManipulatorController.h"
#include "KrakenCamera/FollowingCameraComponent.h"
#include "ROSConDemoSystemComponent.h"
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

namespace ROSConDemo
{
    class ROSConDemoModuleInterface : public AZ::Module
    {
    public:
        AZ_RTTI(ROSConDemoModuleInterface, "{bcfb7709-ca93-8b50-b9c4-caaf1853e479}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ROSConDemoModuleInterface, AZ::SystemAllocator);

        ROSConDemoModuleInterface()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                { ROSConDemoSystemComponent::CreateDescriptor(),
                  AppleKraken::ApplePickerComponent::CreateDescriptor(),
                  AppleKraken::GatheringRowComponent::CreateDescriptor(),
                  AppleKraken::KrakenEffectorComponent::CreateDescriptor(),
                  AppleKraken::FruitStorageComponent::CreateDescriptor(),
                  AppleKraken::DemoStatisticsComponent::CreateDescriptor(),
                  AppleKraken::ManipulatorController::CreateDescriptor(),
                  AppleKraken::FollowingCameraComponent::CreateDescriptor(),
                });
        }

        //! Add required SystemComponents to the SystemEntity.
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROSConDemoSystemComponent>(),
            };
        }
    };
} // namespace ROS2
