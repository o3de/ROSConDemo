

--[[
    This is a proof-of-concept implementation of vehicle control physics. 
    It allows controlling the speed and steering angle of the vehicle 
    using the keyboard.
--]]


function clamp(value, lower_limit, upper_limit)
    if lower_limit == upper_limit then
        return value
    else
        if value > upper_limit then
            return upper_limit
        elseif value < lower_limit then
            return lower_limit
        else
            return value
        end
    end
end

local PID = {}
PID.__index = PID

function PID.new(kp, kd, ki, limit_output)
    local self = setmetatable({}, PID)
    self._kp = kp
    self._kd = kd
    self._ki = ki
    self._preError = 0
    self._integral = 0
    self._limit = limit_output
    return self
end


function PID:Reset()
    self._preError = 0
    self._integral = 0
end

--function PID:Calculate(dt, setpoint, pv)
--    local err = (setpoint - pv)

function PID:Calculate(dt, err)
    local pOut = (self._kp * err)
    self._integral = self._integral + (err * dt)
    local iOut = (self._ki * self._integral)
    local deriv = ((err - self._preError) / dt)
    local dOut = (self._kd * deriv)
    output = (pOut + iOut + dOut)
    self._preError = err
    if self._limit>0.0 then
        output = clamp(output, -self._limit, self._limit)
    end
    return output
end




local manipulator_control = 
{
    Properties =
    {
    -- Entities that are needed by the control

            segment1 = { default = EntityId() },
            segment2 = { default = EntityId() },
            segment3 = { default = EntityId() },
            segment4 = { default = EntityId() },

            gripper = { default = EntityId() },
            rest = { default = EntityId() },
            debug = { default = EntityId() },

            --apple1 = { default = EntityId() },
            --apple2 = { default = EntityId() },



    }
}




--[[
    enum class EffectorState
    {
        IDLE = 0, //!< Idle state / position, suitable for robot moving around the environment.
        PREPARED = 10, //!< State and position which are ready for picking tasks.
        PICKING = 20, //!< The effector is on its way to pick fruit.
        RETRIEVING = 30, //!< The effector is retrieving a fruit to storage position.
        INVALID = -1 //!< Invalid state. Requires an additional context that could help user understand what happened. @see PickingState.
    };

]]

State = {
    IDLE = 0,  --!< Idle state / position, suitable for robot moving around the environment.
    PREPARED = 10, --!< State and position which are ready for picking tasks.
    PICKING_BASE = 20, --!< The effector is on its way to pick fruit.
    PICKING_NOSE = 25, --!< The effector is on its way to pick fruit.
    RETRIEVING_NOSE = 30, --!< The effector is retrieving a fruit to storage position.
    RETRIEVING_BASE = 35, --!< The effector is retrieving a fruit to storage position.
    INVALID = -1, --!< Invalid state. Requires an additional context that could help user understand what happened. @see PickingState.
}



function manipulator_control:OnActivate()
     -- Activation Code

     Debug.Log("-- Joint test initialization")

     self.tickBusHandler = TickBus.CreateHandler(self)
     self.tickBusHandler:Connect()    
     
     self.InputNotificationBus = InputEventNotificationBus.Connect(self, InputEventNotificationId("manipulator_keyboard_control"))


     self.target_position = {0.0, 0.0, 0.0, 0.0}

     
     self.pid1 = PID.new(5000.0, 500.0, 0.0, 900.0) 
     self.pid2 = PID.new(1000.0, 200.0, 0.0, 400.0) 
     self.pid3 = PID.new(500.0, 200.0, 0.0, 300.0)
     self.pid4 = PID.new(500.0, 100.0, 0.0, 200.0)
     self.gravityThreshold = 0.1
     

     --[[
     self.pid1 = PID.new(600.0, 50.0, 10.0, 90.0) 
     self.pid2 = PID.new(500.0, 30.0, 0.0, 40.0) 
     self.pid3 = PID.new(200.0, 20.0, 0.0, 30.0)
     self.pid4 = PID.new(200.0, 10.0, 0.0, 20.0)
     self.gravityThreshold = 0.01
     ]]



     self.segment1_limits = {-0.6, 0.6}
     self.segment2_limits = {-0.35, 0.35}
     self.segment3_limits = {-0.10, 0.10}
     self.segment4_limits = {-0.10, 0.10}

     self.zeroPose = {nil, nil, nil, nil} -- not used


     -- To prevent violent reactions right after the simulation starts,
     -- we're waiting this ammount of seconds till running the controller
     self.startupWait = 1.0 --[s]

     self.currentApple = self.Properties.apple1

     -- Target translation of each segment. Relative to current position
     self.internalError = {nil, nil, nil, nil}

     -- Target position (x,y,z) in internal reference
     self.targetPos = Vector3(nil, nil, nil)

     -- Target
     self.requestWorldPos = Vector3(nil, nil, nil)
     self.restPos = Vector3(0.0, -0.1, 0.0)



     self.pickingState = State.PREPARED
     self.lastPickingState = self.pickingState

     self.nearZeroThresholds = {0.1, 0.1, 0.1, 0.1}
     self.baseZeroTimeout = 0.1
     self.baseZeroTime = 0.0

     self.noseZeroTimeout = 0.5
     self.noseZeroTime = 0.0

     self.pickingTimeout = 30.0
     self.pickingTime = 0.0




end


function manipulator_control:getSegmentPos(entityid)
    return TransformBus.Event.GetLocalTranslation(entityid)
end



function manipulator_control:_setSegmentPos(entityid, pid, target_pos, axis1, print_debug)

    --[[

    local current_pos = self:getSegmentPos(entityid)

    current_pos = current_pos[axis1]

    target_pos = current_pos + target_pos
    ]]

    local err = target_pos
    local force = pid:Calculate(self.deltaTime, err)

    local impulse = force * self.deltaTime

    local force_vector = Vector3(0.0, 0.0, impulse)

    local tm = TransformBus.Event.GetWorldTM(entityid)

    force_vector = Transform.TransformVector(tm, force_vector)

    --RigidBodyRequestBus.Event.SetLinearVelocity(entityid, force_vector)   
    RigidBodyRequestBus.Event.ApplyLinearImpulse(entityid, force_vector)   

    if print_debug then
        -- local entity_name = GameEntityContextRequestBus.Event.GetEntityName(entityid) --TODO make this work

        --Debug.Log('[ '..tostring(entityid)..'] is: '..string.format("%1.3f",current_pos)..'  should be: '..string.format("%1.3f",target_pos)..'  impulse: '..string.format("%1.3f",force))
        --Debug.Log('is: '..string.format("%1.3f",current_pos)..'  should be: '..string.format("%1.3f",target_pos1)..' (delta:'..string.format("%1.3f",target_pos)..')  impulse: '..string.format("%1.3f",force))
        Debug.Log('is: '..string.format("%1.3f",current_pos)..'  should be: '..string.format("%1.3f",target_pos)..'  impulse: '..string.format("%1.3f",force))
    end

end



function manipulator_control:_setBasePosition()
    self:_setSegmentPos(self.Properties.segment1, self.pid1, self.internalError[1], 'z', false)
    self:_setSegmentPos(self.Properties.segment2, self.pid2, self.internalError[2], 'x', false)

    --return false
end

function manipulator_control:_setNosePosition()
    self:_setSegmentPos(self.Properties.segment3, self.pid3, self.internalError[3], 'y', false)
    self:_setSegmentPos(self.Properties.segment4, self.pid4, self.internalError[4], 'z', false)

    --return false
end


function manipulator_control:_findGripperApplePosition(target_position)
    local tm = TransformBus.Event.GetWorldTM(self.Properties.gripper)
    local gripper_pos = Transform.GetTranslation(tm)
    Transform.Invert(tm)

    local pos = Transform.TransformVector(tm, target_position - gripper_pos)
    --Debug.Log(tostring(pos))
    --rel_pos = Vector3(pos['x'], -pos['y'], -pos['z'])

    TransformBus.Event.SetLocalTranslation(self.Properties.debug, pos) 

    return pos

end


function manipulator_control:_getWorldPosition(target_position)

    return self:_findGripperApplePosition(target_position)

end

function manipulator_control:_setInternalState()

    self.internalError[1] = -self.targetPos['z'] + self.gravityThreshold 
    self.internalError[2] = self.targetPos['x'] 
    self.internalError[3] = self.targetPos['y'] 
    self.internalError[4] = self.targetPos['y']

end

function manipulator_control:_isNearZero(segment)
    -- self.nearZeroThresholds

    if math.abs(self.internalError[segment]) < self.nearZeroThresholds[segment] then
        return true
    else
        return false
    end
end


function manipulator_control:_printDebugInfo()
    if self.lastPickingState ~= self.pickingState then
        self.lastPickingState = self.pickingState


        if self.pickingState == State.IDLE then
            txt = 'IDLE'

        elseif self.pickingState == State.PREPARED then
            txt = 'PREPARED'

        elseif self.pickingState == State.PICKING_BASE then
            txt = 'PICKING_BASE'

        elseif self.pickingState == State.PICKING_NOSE then
            txt = 'PICKING_NOSE'

        elseif self.pickingState == State.RETRIEVING_NOSE then
            txt = 'RETRIEVING_NOSE'

        elseif self.pickingState == State.RETRIEVING_BASE then
            txt = 'RETRIEVING_BASE'

        elseif self.pickingState == State.INVALID then
            txt = 'INVALID'

        else
            -- ??
        end

        Debug.Log("Changed state to: ["..txt.."]")

    end
end



function manipulator_control:_orchestrator()


    ----------------------------------------------------------

    if self.pickingState == State.INVALID then
        self.requestWorldPos = TransformBus.Event.GetWorldTranslation(self.Properties.rest)
        self.targetPos = self:_getWorldPosition(self.requestWorldPos)
    end


    ----------------------------------------------------------

    if self.pickingState == State.RETRIEVING_BASE then
        self.requestWorldPos = TransformBus.Event.GetWorldTranslation(self.Properties.rest)
        self.targetPos = self:_getWorldPosition(self.requestWorldPos)
        if (self:_isNearZero(1) and self:_isNearZero(2) and self:_isNearZero(3) and self:_isNearZero(4)) then
            self.pickingState = State.PREPARED
        end
    end


    ----------------------------------------------------------

    if self.pickingState == State.RETRIEVING_NOSE then

        self.requestWorldPos = TransformBus.Event.GetWorldTranslation(self.Properties.rest)
        self.targetPos['y'] = self:_getWorldPosition(self.requestWorldPos)['y']

        if (self:_isNearZero(3) and self:_isNearZero(4)) then
            self.noseZeroTime = self.noseZeroTime + self.deltaTime
            if self.noseZeroTime > self.noseZeroTimeout then
                self.noseZeroTime = 0.0
                self.pickingState = State.RETRIEVING_BASE
            end
        end
    end

    ----------------------------------------------------------

    if self.pickingState == State.PREPARED then
        self.requestWorldPos = TransformBus.Event.GetWorldTranslation(self.Properties.rest)
        self.targetPos = self:_getWorldPosition(self.requestWorldPos)
    end

    ----------------------------------------------------------

    if self.pickingState == State.PICKING_NOSE then

        self.targetPos = self:_getWorldPosition(self.requestWorldPos)

        if (self:_isNearZero(3) and self:_isNearZero(4)) then
            self.noseZeroTime = self.noseZeroTime + self.deltaTime
            if self.noseZeroTime > self.noseZeroTimeout then
                self.noseZeroTime = 0.0
                self.pickingTime = 0.0
                self:Retract()
                --self.pickingState = State.RETRIEVING_NOSE
            end
        end

        self.pickingTime = self.pickingTime + self.deltaTime
        if self.pickingTime > self.pickingTimeout then
            self.pickingState = State.INVALID
            self.pickingTime = 0.0
        end
    end

    ----------------------------------------------------------

    if self.pickingState == State.PICKING_BASE then

        self.targetPos = self:_getWorldPosition(self.requestWorldPos)
        self.targetPos['y'] = self.restPos['y']

        if (self:_isNearZero(1) and self:_isNearZero(2)) then
            self.baseZeroTime = self.baseZeroTime + self.deltaTime
            if self.baseZeroTime > self.baseZeroTimeout then
                self.baseZeroTime = 0.0
                self.pickingState = State.PICKING_NOSE
            end
        end

        self.pickingTime = self.pickingTime + self.deltaTime
        if self.pickingTime > self.pickingTimeout then
            self.pickingState = State.INVALID
            self.pickingTime = 0.0
        end
    end

    ----------------------------------------------------------



    self:_setInternalState()
    self:_setBasePosition()
    self:_setNosePosition()

    self:_printDebugInfo()

end

function manipulator_control:PickApple(apple_pos)
    --if self.pickingState == state.PREPARED then

    self.requestWorldPos = apple_pos
    self.pickingState = State.PICKING_BASE

    Debug.Log("Picking apple at "..tostring(apple_pos))

    --end
end


function manipulator_control:Retract()
    --if self.pickingState == state.PREPARED then

    --self.requestWorldPos = rest_pos
    self.pickingState = State.RETRIEVING_NOSE

    --Debug.Log("Retracting to "..tostring(rest_pos))

    --end
end

function manipulator_control:getStatus()
    return self.pickingState
end


--function manipulator_control:OnHeld  (value)
function manipulator_control:OnPressed(value)
    -- Keypress actions

    -- Process key press

    --[[
    -- UP --
    if value == 8.0 then
        --Debug.Log('up')
        --self.target_position[1] = 0.7
        self.target_position[1] = self.segment1_limits[2]

    end
    -- DOWN --
    if value == 2.0 then
        --Debug.Log('down')

        --self.target_position[1] = -0.7
        self.target_position[1] = self.segment1_limits[1]

    end
    -- LEFT --
    if value == 4.0 then
        --self.target_position[2] = 0.35
        self.target_position[2] = self.segment2_limits[1]


    end
    -- RIGHT --
    if value == 6.0 then
        --self.target_position[2] = -0.35
        self.target_position[2] = self.segment2_limits[2]

    end

    if value == 5.0 then
        self.target_position[1] = 0.0
        self.target_position[2] = 0.0
        self.target_position[3] = 0.0
        self.target_position[4] = 0.0

    end


    if value == 7.0 then
        self.target_position[3] = self.segment3_limits[2]
        self.target_position[4] = self.segment4_limits[1]

    end

    if value == 9.0 then
        self.target_position[3] = self.segment3_limits[1]
        self.target_position[4] = self.segment4_limits[2]
    end

    ]]


    --[[
    if value == 1.0 then
        self.currentApple = self.Properties.apple1
        self:PickApple(TransformBus.Event.GetWorldTranslation(self.currentApple))
    end

    if value == 3.0 then
        self.currentApple = self.Properties.apple2
        self:PickApple(TransformBus.Event.GetWorldTranslation(self.currentApple))
    end 
    ]]   

end

function manipulator_control:OnDeactivate()
     -- Deactivation Code
end

-- This callback is called every frame by the tick bus after this entity activates
function manipulator_control:OnTick(deltaTime, timePoint)

    self.deltaTime = deltaTime

    if self.Properties.segment1~=nil then

        if self.zeroPose[1]==nil then
            self.zeroPose = { -- TODO not used! Cane be removed?
                self:getSegmentPos(self.Properties.segment1),
                self:getSegmentPos(self.Properties.segment2),
                self:getSegmentPos(self.Properties.segment3),
                self:getSegmentPos(self.Properties.segment4),
            }

        else
            if self.startupWait > 0 then
                self.startupWait = self.startupWait -self.deltaTime
                --Debug.Log(string.format("%1.3f",self.startupWait))
                --self:Retract(TransformBus.Event.GetWorldTranslation(self.Properties.rest))
            else
                --self:Retract(TransformBus.Event.GetWorldTranslation(self.Properties.rest))
                self:_orchestrator()
            end
        end
    end


end



return manipulator_control
