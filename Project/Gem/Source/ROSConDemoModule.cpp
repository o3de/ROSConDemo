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
#include "Manipulator/ManipulatorJoySubscriber.h"
#include "KrakenCamera/FollowingCameraComponent.h"
#include "KrakenCamera/CameraJoySubscriber.h"

#include "ROSConDemoSystemComponent.h"
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

namespace ROSConDemo
{
    class ROSConDemoModule : public AZ::Module
    {
    public:
        AZ_RTTI(ROSConDemoModule, "{E38575E4-7D2F-4617-B938-416E8C1C07B4}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ROSConDemoModule, AZ::SystemAllocator, 0);

        ROSConDemoModule()
            : AZ::Module()
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
                  AppleKraken::ManipulatorJoySubscriber::CreateDescriptor(),
                  AppleKraken::CameraJoySubscriber::CreateDescriptor() });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROSConDemoSystemComponent>(),
            };
        }
    };
} // namespace ROSConDemo

AZ_DECLARE_MODULE_CLASS(Gem_ROSConDemo, ROSConDemo::ROSConDemoModule)
