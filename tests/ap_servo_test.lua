local servo = dofile("plugins/xtlua_keysystems/scripts/B747.19.xt.hydraulicsmodel/B747.19.xt.ap_servo.lua")
local tests_run = 0
local function check(value, message)
    tests_run = tests_run + 1
    assert(value, message)
end
local function near(actual, expected, tolerance, message)
    check(type(actual) == "number" and math.abs(actual - expected) <= tolerance, message)
end

local s = servo.new({disagreement_limit = 0.2, rate = 0.5})
check(s.state == servo.DISENGAGED, "new servo is disengaged")
near(s:synchronize(0.4), 0.4, 0.001, "synchronizes to control position")
near(servo.vote({-0.7, 0.1, 0.4}, {true, true, true}), 0.1, 0.001, "triple median")
near(servo.vote({0.8, 0.2, 0}, {true, true, false}), 0.2, 0.001, "dual least value")
check(servo.vote({0.8, 0.2, 0}, {false, false, false}) == nil, "hydraulic gating")
local output = s:update(1, 1, {true, true, true}, true, 0.4)
near(output, 0.9, 0.001, "authority is rate bounded")
output = s:update(1, 1, {true, true, true}, true, 0.9)
near(output, 1.0, 0.001, "bounded output reaches command")
near(s:update(0.1, 1, {true, true, true}, true, 1), 0.5, 0.001,
    "rate limits a reversal toward a small command")
check(s:update(0.1, 1, {false, false, false}, true, 0.1) == nil, "drops out without hydraulics")
check(s.state == servo.DISENGAGED, "hydraulic loss disengages the servo")
check(s:update(0.1, 1, {true, true, true}, false, 0.1) == nil and s.state == servo.DISENGAGED,
    "disengagement is fail safe")
local _, disagreement = servo.new({disagreement_limit = 0.1}):update({0, 0.5, 0}, 1,
    {true, true, true}, true, 0)
check(disagreement, "channel disagreement is detected")
print("AP servo tests passed (" .. tests_run .. ")")
