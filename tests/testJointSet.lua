#!/usr/bin/env th
local js = require 'xamlamoveit.datatypes'.JointSet
local luaunit = require 'luaunit'
TestJointSet = {}

local set_a =  js({'A', 'AB', 'AC'})
local set_b =  js({'AB', 'AC', 'A'})
local set_c =  js({'AC', 'A'})
local set_d =  js({'AB'})

function TestJointSet:testEq()
    luaunit.assertTrue(set_a == set_b)
    luaunit.assertTrue(set_b == set_a)
    luaunit.assertFalse(set_a == set_c)
    luaunit.assertFalse(set_c == set_a)
end

function TestJointSet:testLt()
    luaunit.assertTrue(set_c < set_a)
    luaunit.assertTrue(set_c < set_a)
    luaunit.assertTrue(set_b > set_c)
end

function TestJointSet:testAdd()
    local result_A = set_c + set_d
    local result_B = set_d + set_c
    luaunit.assertTrue(result_A == set_a)
    luaunit.assertTrue(result_B == set_a)
end


os.exit(luaunit.LuaUnit.run())