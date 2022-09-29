/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ApplePickerComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
// #include <ROS2/ROS2Bus.h>

namespace AppleKraken
{
    void ApplePickerComponent::StartAutomatedOperation() // Probably to be connected to another bus
    {
    }

    void ApplePickerComponent::Activate()
    {
    }

    void ApplePickerComponent::Deactivate()
    {
    }

    void ApplePickerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ApplePickerComponent, AZ::Component>()->Version(1);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ApplePickerComponent>("Apple picking component", "A demo component for apple picking")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken");
            }
        }
    }

    void ApplePickerComponent::ApplePicked(AZ::EntityId appleId)
    {
        AZ_TracePrintf("ApplePicker", "Picked apple id %s\n", appleId.ToString().c_str());
    }

    void ApplePickerComponent::AppleRetrieved()
    {
        AZ_TracePrintf("ApplePicker", "An apple has been retrieved and stored\n");
    }

    void ApplePickerComponent::PickingFailed(AZ::EntityId appleId, const AZStd::string& reason)
    {
        AZ_TracePrintf("ApplePicker", "Picking apple %s failed due to: %s\n", appleId.ToString().c_str(), reason.c_str());
    }
} // namespace AppleKraken