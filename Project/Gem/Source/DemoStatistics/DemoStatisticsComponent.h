/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "DemoStatisticsNotifications.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>

namespace AppleKraken
{
    //! Component responsible for holding simulation statistics such as failed and picked apples, globally.
    class DemoStatisticsComponent
        : public AZ::Component
        , private DemoStatisticsNotificationBus::Handler
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(DemoStatisticsComponent, "{C421B635-8B7A-479A-8617-86F5F1ACC4AE}");

        DemoStatisticsComponent() = default;
        ~DemoStatisticsComponent() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
        void DisplayNumberOfApples();
        void AddApple(const AppleEvent& appleEvent) override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        bool HasTag(const AppleEvent& appleEvent, const AZStd::string& tag);
        bool IsFailed(const Tags& tags);
        bool IsAutomated(const Tags& tags);

        void UpdateTextField(const AZStd::string& fieldName, const AZStd::string& label, uint16_t counter);
        const AZStd::string m_appleGatheredElementName = "ApplesGathered";
        const AZStd::string m_appleFailedElementName = "ApplesFailed";
        AZ::EntityId m_uiEntity;
        uint16_t m_applesGathered = 0;
        uint16_t m_applesFailed = 0;
    };
} // namespace AppleKraken
