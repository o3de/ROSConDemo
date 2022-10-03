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

namespace AppleKraken
{
    namespace Internal
    {
        AZStd::string TaskString(const PickAppleTask& task)
        {
            if (!task.m_appleEntityId.IsValid())
            {
                return "|Task for an unspecified apple|";
            }

            return AZStd::string::format("|Task for entity id %s|", task.m_appleEntityId.ToString().c_str());
        }

        AZStd::string CurrentTaskString(const AZStd::queue<PickAppleTask>& taskQueue)
        {
            if (taskQueue.empty())
            {
                return "|No task, pick queue empty!|";
            }

            const auto& currentTask = taskQueue.front();
            return TaskString(currentTask);
        }
    } // namespace Internal

    void ApplePickerComponent::StartAutomatedOperation()
    {
    }

    float ApplePickerComponent::ReportProgress()
    {
        return 0.0f;
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

    void ApplePickerComponent::ApplePicked()
    {
        AZ_TracePrintf("ApplePicker", "%s. Picked apple\n", Internal::CurrentTaskString(m_currentAppleTasks).c_str());
    }

    void ApplePickerComponent::AppleRetrieved()
    {
        AZ_TracePrintf(
            "ApplePicker", "%s. An apple has been retrieved and stored\n", Internal::CurrentTaskString(m_currentAppleTasks).c_str());
    }

    void ApplePickerComponent::PickingFailed(const AZStd::string& reason)
    {
        AZ_TracePrintf(
            "ApplePicker", "%s. Picking failed due to: %s\n", Internal::CurrentTaskString(m_currentAppleTasks).c_str(), reason.c_str());
    }
} // namespace AppleKraken