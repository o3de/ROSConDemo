/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GatheringRowComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace AppleKraken
{
    void GatheringRowComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GatheringRowComponent, AZ::Component>()
                ->Version(3)
                ->Field("Start", &GatheringRowComponent::m_start)
                ->Field("End", &GatheringRowComponent::m_end)
                ->Field("PoseOffset", &GatheringRowComponent::m_poseOffset)
                ->Field("TreeCount", &GatheringRowComponent::m_appleTreeCount);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<GatheringRowComponent>("Gathering Row Component", "Poses (points with orientation) suitable for apple gathering")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GatheringRowComponent::m_start, "Start", "Entity with the start pose")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GatheringRowComponent::m_end, "End", "Entity with the end pose")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GatheringRowComponent::m_poseOffset,
                        "Offset",
                        "Pose offset for each point (depends on robot)")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GatheringRowComponent::m_appleTreeCount,
                        "Tree count",
                        "How many trees are in the row")
                    ->Attribute(AZ::Edit::Attributes::Min, 1);
            }
        }
    }

    void GatheringRowComponent::Activate()
    {
        GatheringRowRequestBus::Handler::BusConnect();
    }

    void GatheringRowComponent::Deactivate()
    {
        GatheringRowRequestBus::Handler::BusDisconnect();
        m_gatheringPoses.clear();
    }

    void GatheringRowComponent::ComputeGatheringPoses()
    {
        if (!m_start.IsValid() || !m_end.IsValid())
        {
            AZ_Error("GatheringRowComponent", false, "ComputeGatheringPoses: unable to proceed without both start and end entity set");
            return;
        }

        if (m_appleTreeCount < 1)
        {
            AZ_Error("GatheringRowComponent", false, "ComputeGatheringPoses: unable to proceed with apple tree count less than 1");
            return;
        }

        GatheringPoses allPoses;
        AZ::Transform startPose;
        AZ::Transform endPose;
        AZ::TransformBus::EventResult(startPose, m_start, &AZ::TransformBus::Events::GetWorldTM);
        AZ::TransformBus::EventResult(endPose, m_end, &AZ::TransformBus::Events::GetWorldTM);

        // Simplification - we assume same orientations along the way
        const auto rowVector = endPose.GetTranslation() - startPose.GetTranslation();
        for (int i = 0; i < m_appleTreeCount; ++i)
        {
            AZ::Transform gatheringPoint = startPose;
            const float scale = static_cast<float>(i) / (m_appleTreeCount - 1);
            gatheringPoint.SetTranslation(gatheringPoint.GetTranslation() + m_poseOffset + rowVector * scale);
            m_gatheringPoses.emplace_back(gatheringPoint);
        }
    }
} // namespace AppleKraken
