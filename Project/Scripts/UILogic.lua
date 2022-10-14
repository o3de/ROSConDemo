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
local UILogic = {
    Properties = {
        Debug = false,
        QuitInputEvent = "Quit",
        ResetInputEvent = "Reset",
        CtrlInputEvent = "Ctrl"
    },
    InputHandlers = {
        Quit = {},
        Reset = {},
        Ctrl = {}
    },
    GameplayEventHandlers = {
        OnCancel = {},
        OnQuit = {}
    }
}

function UILogic:OnActivate()
    self.ctrlPressed = false
    self.resetPressed = false
    self.quitCanvasVisible = false

    for event, handler in pairs(self.InputHandlers) do
        local id = InputEventNotificationId(event)
        handler.busHandler = InputEventNotificationBus.Connect(handler, id)
        handler.component = self
    end
    for event, handler in pairs(self.GameplayEventHandlers) do
        local id = GameplayNotificationId(EntityId(0), event, "float")
        handler.busHandler = GameplayNotificationBus.Connect(handler, id)
        handler.component = self
    end
end

function UILogic:SetCanvasVisible(visible)
    if visible then
        if self.quitCanvasVisible == false then
            self.quitCanvasVisible = true
            -- show cursor
            while UiCursorBus.Broadcast.IsUiCursorVisible() == false do
                UiCursorBus.Broadcast.IncrementVisibleCounter()
            end
            FlyCameraInputBus.Broadcast.SetIsEnabled(false)    
            UiCanvasAssetRefBus.Event.LoadCanvas(self.entityId)
        end
    else
        self.quitCanvasVisible = false

        -- hide cursor
        UiCursorBus.Broadcast.DecrementVisibleCounter()
        FlyCameraInputBus.Broadcast.SetIsEnabled(true)
        UiCanvasAssetRefBus.Event.UnloadCanvas(self.entityId)
    end
end

function UILogic.GameplayEventHandlers.OnCancel:OnEventBegin(value)
    self.component:SetCanvasVisible(false)
end

function UILogic.GameplayEventHandlers.OnQuit:OnEventBegin(value)
    Debug.Log("Exiting application")
    ConsoleRequestBus.Broadcast.ExecuteConsoleCommand("quit")
end

function UILogic:Reset()
    if self.resetPressed and self.ctrlPressed then
        Debug.Log("Re-loading level")
        ROSConDemoRequestBus.Broadcast.ReloadLevel();
        -- LoadLevel command will reload the error but currently has graphic
        -- issues so a custom ReloadLevel command above is used
        -- ConsoleRequestBus.Broadcast.ExecuteConsoleCommand("LoadLevel main")
    end
end

function UILogic.InputHandlers.Quit:OnPressed(value)
    self.component:SetCanvasVisible(true)
end

function UILogic.InputHandlers.Reset:OnPressed(value)
    self.component.resetPressed = true
    self.component:Reset()
end

function UILogic.InputHandlers.Reset:OnReleased(value)
    self.component.resetPressed = false
end

function UILogic.InputHandlers.Ctrl:OnPressed(value)
    self.component.ctrlPressed = true
end

function UILogic.InputHandlers.Ctrl:OnReleased(value)
    self.component.ctrlPressed = false
end

function UILogic:OnDeactivate()
    for event, handler in pairs(self.InputHandlers) do
        handler.busHandler:Disconnect()
    end
    for event, handler in pairs(self.GameplayEventHandlers) do
        handler.busHandler:Disconnect()
    end
end

return UILogic

