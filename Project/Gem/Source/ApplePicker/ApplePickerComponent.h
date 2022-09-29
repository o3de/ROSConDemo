/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ApplePickingNotifications.h"
#include "ApplePickingRequests.h"
#include <AzCore/Component/Component.h>
// #include <vision_msgs/msgs/detection_3d_array.h>

namespace AppleKraken
{
    //! The component also acts as a ground-truth detector
    // using DetectionsMessage = vision_msgs::msgs::Detection3DArray;

    struct AppleTask
    {
        AZ::EntityId m_appleEntityId;
        AZ::Aabb m_appleBoundingBox;
    };

    //! Demo component handling orchestration of apple picking
    class ApplePickerComponent
        : public AZ::Component
        , private ApplePickingNotificationBus::Handler // Probably could use TickBus as well for timeouts
    {
    public:
        AZ_COMPONENT(ApplePickerComponent, "{E9E83A4A-31A4-4E7A-AF88-7565AC8B9F27}", AZ::Component);
        ApplePickerComponent() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

        void StartAutomatedOperation();

    private:
        void ApplePicked(AZ::EntityId appleId) override;
        void AppleRetrieved() override;
        void PickingFailed(AZ::EntityId appleId, const AZStd::string& reason) override;

        AZ::Obb m_gatheringArea;
        AZStd::queue<AppleTask> m_currentAppleTasks;
    };
} // namespace AppleKraken
