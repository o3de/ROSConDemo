/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "DemoStatisticsComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>
#include <LyShine/Bus/UiCanvasBus.h>
#include <LyShine/Bus/UiTextBus.h>
#include <LyShine/Bus/World/UiCanvasRefBus.h>

namespace AppleKraken
{
    void DemoStatisticsComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        DemoStatisticsNotificationBus::Handler::BusConnect();
        m_applesGathered = 0;
        m_applesFailed = 0;
    }

    void DemoStatisticsComponent::Deactivate()
    {
        DemoStatisticsNotificationBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void DemoStatisticsComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<DemoStatisticsComponent, AZ::Component>()->Version(1)->Field(
                "UiEntity", &DemoStatisticsComponent::m_uiEntity);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<DemoStatisticsComponent>("Demo Statistics", "Demo statistics for picked apples")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Handles counting of interesting events and their UI")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &DemoStatisticsComponent::m_uiEntity,
                        "Entity with Ui canvas reference ",
                        "Element which will display the text of statistics");
            }
        }
    }

    void DemoStatisticsComponent::AddApple(const AppleEvent& appleEvent)
    {
        if (IsFailed(appleEvent))
        {
            m_applesFailed++;
            return;
        }

        m_applesGathered++;
    }

    bool DemoStatisticsComponent::IsFailed(const Tags& tags)
    {
        return HasTag(tags, kPickingFailedEventTag);
    }

    // TODO Currently not used, but will be soon
    bool DemoStatisticsComponent::IsAutomated(const Tags& tags)
    {
        return HasTag(tags, kPickingAutomatedEventTag);
    }

    bool DemoStatisticsComponent::HasTag(const AppleEvent& appleEvent, const AZStd::string& tag)
    {
        return AZStd::find(appleEvent.cbegin(), appleEvent.cend(), tag) != appleEvent.cend();
    }

    void DemoStatisticsComponent::DisplayNumberOfApples()
    {
        UpdateTextField(m_appleGatheredElementName, "Apples gathered: ", m_applesGathered);
        UpdateTextField(m_appleFailedElementName, "Attempts failed:", m_applesFailed);
    }

    void DemoStatisticsComponent::UpdateTextField(const AZStd::string& fieldName, const AZStd::string& label, uint16_t counter)
    {
        AZ::EntityId uiCanvas;
        EBUS_EVENT_ID_RESULT(uiCanvas, m_uiEntity, UiCanvasRefBus, GetCanvas);
        if (!uiCanvas.IsValid())
        {
            AZ_Warning("DemoStatisticsComponent", false, "%s does not have a UiCanvasRef attached\n", m_uiEntity.ToString().c_str());
            return;
        }

        AZ::EntityId uiTextEntity;
        UiCanvasBus::EventResult(uiTextEntity, uiCanvas, &UiCanvasInterface::FindElementEntityIdByName, fieldName);
        if (!uiTextEntity.IsValid())
        {
            AZ_Warning("DemoStatisticsComponent", false, "ui canvas doest not have a %s field\n", fieldName.c_str());
            return;
        }
        UiTextBus::Event(uiTextEntity, &UiTextInterface::SetText, AZStd::string::format("%s %d", label.c_str(), counter));
    }

    void DemoStatisticsComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        DisplayNumberOfApples();
    }
} // namespace AppleKraken
