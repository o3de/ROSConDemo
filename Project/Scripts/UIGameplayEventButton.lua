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
	
	if self.Properties.InputEvent ~= "" then
		local id = InputEventNotificationId(self.Properties.InputEvent)
		self.inputHandler = InputEventNotificationBus.Connect(self, id)
	end
end

function UIGameplayEventButton:OnPressed(value)
    GameplayNotificationBus.Event.OnEventBegin(GameplayNotificationId(EntityId(0), self.Properties.Event, "float"), self.Properties.Value)
end


function UIGameplayEventButton:OnButtonClick()
    Debug.Log("Sending " .. tostring(self.Properties.Event))
    GameplayNotificationBus.Event.OnEventBegin(GameplayNotificationId(EntityId(0),self.Properties.Event, "float"), self.Properties.Value)
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