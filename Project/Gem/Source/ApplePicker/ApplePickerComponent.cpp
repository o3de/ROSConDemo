/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ApplePickerComponent.h"
#include "ApplePickingRequests.h"
#include <AzCore/EBus/Event.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <ROS2/Frame/ROS2FrameComponent.h>

using namespace ROS2;

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

    bool ApplePickerComponent::IsBusy() const
    {
        if (!m_currentAppleTasks.empty())
        {   // busy - still has tasks in queue
            return true;
        }

        // There are no apple tasks, but the effector might not be idle or prepared yet
        PickingState pickingState;
        ApplePickingRequestBus::EventResult(pickingState, m_effectorEntityId, &ApplePickingRequests::GetEffectorState);
        if (pickingState.m_effectorState != EffectorState::IDLE && pickingState.m_effectorState != EffectorState::PREPARED)
        {   // Effector is not ready - still transitioning to a state. This should be a rare occurrence.
            AZ_Warning("ApplePicker", false, "Task queue empty but apple picker is busy since the effector is working");
            return true;
        }
        return false;
    }

    void ApplePickerComponent::ProcessTriggerServiceCall(const TriggerRequest req, TriggerResponse resp)
    {
        // TODO - also, perhaps add a check whether Kraken is in gathering position, immobile etc.
        if (IsBusy())
        {
            resp->success = false;
            resp->message = "Unable to accept request - apple picker is currently busy";
            return;
        }

        resp->success = true;
        resp->message = "Command accepted, picking operation started";
        StartAutomatedOperation();
        return;
    }

    void ApplePickerComponent::StartAutomatedOperation()
    {
        if (IsBusy())
        {
            AZ_Error("ApplePicker", false, "Tasks still in progress for current picking!");
            return;
        }

        // Get effector reach
        AZ::Obb effectorRangeGlobalBox;
        ApplePickingRequestBus::EventResult(effectorRangeGlobalBox, m_effectorEntityId, &ApplePickingRequests::GetEffectorReachArea);

        // Find out apples within the reach
        QueryEnvironmentForAllApplesInBox(effectorRangeGlobalBox);

        // Tell effector to prepare for picking
        ApplePickingRequestBus::Event(m_effectorEntityId, &ApplePickingRequests::PrepareForPicking);
    }

    void ApplePickerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        // TODO handle timeouts and incoming commands
    }

    float ApplePickerComponent::ReportProgress()
    {
        // TODO (minor) - take into consideration current task progress (effector state)
        if (m_initialTasksSize == 0)
        {
            AZ_Warning("ApplePicker", false, "ReportProgress reporting 1 since no apples were found in the call");
            return 1.0f;
        }

        return 1.0f - (m_currentAppleTasks.size() / m_initialTasksSize);
    }

    void ApplePickerComponent::Activate()
    {
        m_effectorEntityId = GetEntityId(); // TODO - remove this once we expose this field
        ApplePickingNotificationBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();

        auto ros2Node = ROS2Interface::Get()->GetNode();
        auto robotNamespace = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity())->GetNamespace();
        auto topic = ROS2Names::GetNamespacedName(robotNamespace, m_triggerServiceTopic);
        m_triggerService = ros2Node->create_service<std_srvs::srv::Trigger>(
            m_triggerServiceTopic.c_str(),
            [this](const TriggerRequest request, TriggerResponse response)
            {
                this->ProcessTriggerServiceCall(request, response);
            });
    }

    void ApplePickerComponent::Deactivate()
    {
        m_triggerService.reset();
        AZ::TickBus::Handler::BusDisconnect();
        ApplePickingNotificationBus::Handler::BusDisconnect();
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

    void ApplePickerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    void ApplePickerComponent::EffectorReadyForPicking()
    {
        PickNextApple();
    }

    void ApplePickerComponent::ApplePicked()
    {
        if (m_currentAppleTasks.empty())
        {
            AZ_Error("ApplePicker", false, "ApplePicked called but no current task");
            return;
        }
        AZ_TracePrintf("ApplePicker", "%s. Picked apple\n", Internal::CurrentTaskString(m_currentAppleTasks).c_str());
    }

    void ApplePickerComponent::AppleRetrieved()
    {
        if (m_currentAppleTasks.empty())
        {
            AZ_Error("ApplePicker", false, "AppleRetrieved called but no current task");
            return;
        }
        AZ_TracePrintf(
            "ApplePicker", "%s. An apple has been retrieved and stored\n", Internal::CurrentTaskString(m_currentAppleTasks).c_str());
        m_currentAppleTasks.pop();
        PickNextApple();
    }

    void ApplePickerComponent::PickingFailed(const AZStd::string& reason)
    { // TODO - refactor common code (debugs, checks)
        if (m_currentAppleTasks.empty())
        {
            AZ_Error("ApplePicker", false, "PickingFailed called but no current task");
            return;
        }
        AZ_TracePrintf(
            "ApplePicker", "%s. Picking failed due to: %s\n", Internal::CurrentTaskString(m_currentAppleTasks).c_str(), reason.c_str());
        m_currentAppleTasks.pop();
        PickNextApple();
    }

    void ApplePickerComponent::PickNextApple()
    {
        AZ_TracePrintf("ApplePicker", "Pick next apple");
        if (!m_currentAppleTasks.empty())
        { // Get another apple!
            ApplePickingRequestBus::Event(m_effectorEntityId, &ApplePickingRequests::PickApple, m_currentAppleTasks.front());
            return;
        }
    }

    void ApplePickerComponent::QueryEnvironmentForAllApplesInBox(const AZ::Obb& /*globalBox*/)
    {
        // TODO - query environment

        // Debug
        for (int i = 0; i < 5; ++i)
        {
            PickAppleTask emptyTask;
            m_currentAppleTasks.push(emptyTask);
        }
    }

} // namespace AppleKraken