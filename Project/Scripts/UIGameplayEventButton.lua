----------------------------------------------------------------------------------------------------
--
-- Copyright (c) Contributors to the Open 3D Engine Project.
-- For complete copyright and license terms please see the LICENSE at the root of this distribution.
--
-- SPDX-License-Identifier: Apache-2.0 OR MIT
--
--
--
----------------------------------------------------------------------------------------------------
local UIGameplayEventButton = {
    Properties = {
        Debug = false,
        Event = { default = "", description = "Event name" },
        Value = { default = "", description = "Event value" },
        InputEvent = { default = "", description = "Optional input event name"}
    }
}

function UIGameplayEventButton:OnActivate()
    self.buttonHandler = UiButtonNotificationBus.Connect(self, self.entityId)
    self.gameplayNotificationId = GameplayNotificationId(EntityId(0), self.Properties.Event, "float")

    if self.Properties.InputEvent ~= "" then
        local id = InputEventNotificationId(self.Properties.InputEvent)
        self.inputHandler = InputEventNotificationBus.Connect(self, id)
    end
end

function UIGameplayEventButton:OnPressed(value)
    GameplayNotificationBus.Event.OnEventBegin(self.gameplayNotificationId, self.Properties.Value)
end

function UIGameplayEventButton:OnButtonClick()
    GameplayNotificationBus.Event.OnEventBegin(self.gameplayNotificationId, self.Properties.Value)
end

function UIGameplayEventButton:OnDeactivate()
    if self.buttonHandler ~= nil then
        self.buttonHandler:Disconnect()
    end
    
    if self.inputHandler ~= nil then
        self.inputHandler:Disconnect()
    end
end


return UIGameplayEventButton

