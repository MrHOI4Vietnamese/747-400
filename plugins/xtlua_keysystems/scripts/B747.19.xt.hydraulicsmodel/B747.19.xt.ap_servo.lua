-- Three-channel autopilot servo voting and response.  Pure Lua 5.1.
local servo = {}
servo.__index = servo

servo.DISENGAGED = 0
servo.ARMED = 1
servo.ENGAGED = 2

local function clamp(value, low, high)
    if value < low then return low end
    if value > high then return high end
    return value
end

local function median(a, b, c)
    if a > b then a, b = b, a end
    if b > c then b, c = c, b end
    if a > b then a, b = b, a end
    return b
end

function servo.new(options)
    options = options or {}
    return setmetatable({
        state = servo.DISENGAGED,
        position = 0,
        disagreement = false,
        disagreement_limit = tonumber(options.disagreement_limit) or 0.15,
        authority = tonumber(options.authority) or 1,
        rate = tonumber(options.rate) or 1.0
    }, servo)
end

function servo:synchronize(position)
    if type(position) == "number" then self.position = clamp(position, -self.authority, self.authority) end
    return self.position
end

function servo:set_state(state, position)
    if state == servo.DISENGAGED then
        self.state = state
    elseif state == servo.ARMED then
        self:synchronize(position)
        self.state = state
    elseif state == servo.ENGAGED then
        self:synchronize(position)
        self.state = state
    end
    return self.state
end

-- Three valid channels use the median.  With two valid channels the ATA
-- least-value selector is used; one valid channel is passed through.
function servo.vote(commands, available)
    local valid = {}
    for i = 1, 3 do
        if available == nil or available[i] then
            if type(commands[i]) == "number" then valid[#valid + 1] = commands[i] end
        end
    end
    if #valid == 0 then return nil, 0, false end
    local disagreement = false
    local limit = 0
    for i = 1, #valid do
        for j = i + 1, #valid do
            if math.abs(valid[i] - valid[j]) > limit then limit = math.abs(valid[i] - valid[j]) end
        end
    end
    disagreement = limit > 0
    if #valid == 3 then return median(valid[1], valid[2], valid[3]), 3, disagreement end
    if #valid == 2 then return math.min(valid[1], valid[2]), 2, disagreement end
    return valid[1], 1, disagreement
end

-- `active` is supplied by the existing AP servos dataref; this module does
-- not create a competing global servos_on state.
function servo:update(command, elapsed, available, active, current_position)
    if not active then
        self:set_state(servo.DISENGAGED)
        return nil, false
    end
    if self.state == servo.DISENGAGED then self:set_state(servo.ARMED, current_position) end
    if self.state == servo.ARMED then self.state = servo.ENGAGED end
    local commands = type(command) == "table" and command or {command, command, command}
    local voted, count, _ = servo.vote(commands, available)
    if voted == nil then
        self:set_state(servo.DISENGAGED)
        self:synchronize(current_position)
        return nil, false
    end
    self.disagreement = false
    if count > 1 then
        local low, high
        for i = 1, 3 do
            if (available == nil or available[i]) and type(commands[i]) == "number" then
                low = low and math.min(low, commands[i]) or commands[i]
                high = high and math.max(high, commands[i]) or commands[i]
            end
        end
        self.disagreement = high - low > self.disagreement_limit
    end
    elapsed = math.max(tonumber(elapsed) or 0, 0)
    local maximum_change = self.rate * elapsed
    local target = clamp(voted, -self.authority, self.authority)
    local previous = self.position
    if maximum_change > 0 then
        local change = clamp(target - previous, -maximum_change, maximum_change)
        self.position = previous + change
    else
        self.position = target
    end
    return self.position, self.disagreement
end

return servo
