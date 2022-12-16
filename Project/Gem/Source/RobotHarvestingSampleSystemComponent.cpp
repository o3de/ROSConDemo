/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "RobotHarvestingSampleSystemComponent.h"
#include "ApplePicker/GatheringRowRequests.h"
#include <AzCore/Component/TickBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ILevelSystem.h>
#include <ISystem.h>

namespace RobotHarvestingSample
{
    void RobotHarvestingSampleSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RobotHarvestingSampleSystemComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RobotHarvestingSampleSystemComponent>("RobotHarvestingSample", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }

        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<RobotHarvestingSampleRequestBus>("RobotHarvestingSampleRequestBus")
                ->Event("ReloadLevel", &RobotHarvestingSampleRequestBus::Events::ReloadLevel);
        }
    }

    void RobotHarvestingSampleSystemComponent::ReloadLevel()
    {
        ISystem* systemInterface = nullptr;
        CrySystemRequestBus::BroadcastResult(systemInterface, &CrySystemRequests::GetCrySystem);
        if(systemInterface && systemInterface->GetILevelSystem())
        {
            ILevelSystem* levelSystem = systemInterface->GetILevelSystem();
            AZStd::string currentLevelName = levelSystem->GetCurrentLevelName();
            levelSystem->UnloadLevel();
            AZ::TickBus::QueueFunction([levelSystem, currentLevelName]() {
                levelSystem->LoadLevel(currentLevelName.c_str());
            });
        }
    }

    void RobotHarvestingSampleSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {   
        provided.push_back(AZ_CRC("RobotHarvestingSampleService"));
    }

    void RobotHarvestingSampleSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("RobotHarvestingSampleService"));
    }

    void RobotHarvestingSampleSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RobotHarvestingSampleSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    RobotHarvestingSampleSystemComponent::RobotHarvestingSampleSystemComponent()
    {
        if (RobotHarvestingSampleInterface::Get() == nullptr)
        {
            RobotHarvestingSampleInterface::Register(this);
        }
    }

    RobotHarvestingSampleSystemComponent::~RobotHarvestingSampleSystemComponent()
    {
        if (RobotHarvestingSampleInterface::Get() == this)
        {
            RobotHarvestingSampleInterface::Unregister(this);
        }
    }

    void RobotHarvestingSampleSystemComponent::ProcessGetPlanServiceCall(const GetPlanRequestPtr req, GetPlanResponsePtr resp)
    {
        AZ::EBusAggregateResults<AppleKraken::GatheringPoses> results;
        AppleKraken::GatheringRowRequestBus::BroadcastResult(results, &AppleKraken::GatheringRowRequests::GetGatheringPoses);

        // remove all empty rows to avoid checking it later
        results.values.erase(
            std::remove_if(
                results.values.begin(),
                results.values.end(),
                [](const auto& row){ return row.empty(); }),
            results.values.end());

        if (results.values.empty())
        {
            // there are no gathering rows containing at least one pose detected
            return;
        }

        auto startTransform = ROS2::ROS2Conversions::FromROS2Pose(req->start.pose);

        auto nearest_row = *std::min_element(
            results.values.begin(),
            results.values.end(),
            [startTranslation = startTransform.GetTranslation()](const auto& row1,const auto& row2)
            {
                if (row1[0].GetTranslation().GetDistance(startTranslation) < row2[0].GetTranslation().GetDistance(startTranslation))
                {
                   return true;
                }
                return false;
            });

        for( const auto& pose : nearest_row)
        {
            geometry_msgs::msg::PoseStamped stampedPose; // TODO - fill in header
            stampedPose.pose = ROS2::ROS2Conversions::ToROS2Pose(pose);
            resp->plan.poses.push_back(stampedPose);
        }
    }

    void RobotHarvestingSampleSystemComponent::Activate()
    {   // TODO - the service should probably only be created in Game Mode
        RobotHarvestingSampleRequestBus::Handler::BusConnect();
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_pathPlanService = ros2Node->create_service<nav_msgs::srv::GetPlan>(
            m_planTopic.c_str(),
            [this](const GetPlanRequestPtr request, GetPlanResponsePtr response)
            {
                this->ProcessGetPlanServiceCall(request, response);
            });
    }

    void RobotHarvestingSampleSystemComponent::Deactivate()
    {
        m_pathPlanService.reset();
        RobotHarvestingSampleRequestBus::Handler::BusDisconnect();
    }
} // namespace RobotHarvestingSample
