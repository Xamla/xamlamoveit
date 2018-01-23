function table.values()
    local r = {}
    for k, v in pairs(t) do
        r[#r + 1] = v
    end
    return r
end

function table.keys(t)
    local r = {}
    for k, v in pairs(t) do
        r[#r + 1] = k
    end
    return r
end

function table.merge(dst, src)
    for k, v in pairs(src) do
        dst[k] = src[k]
    end
    return dst
end

function table.indexof(t, x)
    for i, y in ipairs(t) do
        if y == x then
            return i
        end
    end
    return -1
end

function table.clone(t, deep)
    if not t then
        return nil
    end

    local r = {}
    for k, v in pairs(t) do
        if type(v) == 'table' and deep then
            r[k] = table.clone(v, true)
        else
            r[k] = v
        end
    end
    return r
end

function table.filter(t, condition)
    local r = {}
    for k, v in pairs(t) do
        if condition(v) then
            r[k] = v
        end
    end
    return r
end

function table.tolookup(t, keySelector)
    local m = {}
    for k, x in pairs(t) do
        local key = keySelector(x)
        if m[key] then
            error(stirng.format('Duplicate key: %s', key))
        end
        m[key] = x
    end
    return m
end

function table.map(t, selector)
    local r = {}
    for k, v in pairs(t) do
        r[k] = selector(v)
    end
    return r
end

function table.findIndex(t, condition)
    for i, v in ipairs(t) do
        if condition(v, i) then
            return i
        end
    end
    return -1
end

function table.findIndicesTensor(t, condition)
    local result = torch.ByteTensor(#t):zero()
    for i, v in ipairs(t) do
        if condition(v) then
            result[i] = 1
        end
    end
    return result
end

function table.isSubset(A, B)
    for ia, a in ipairs(A) do
        if table.indexof(B, a) == -1 then
            return false
        end
    end
    return true
end

function table.isSimilar(A, B)
    if #A == #B then
        return table.isSubset(A, B)
    else
        return false
    end
    return true
end

function table.swapKeyValue(t)
    local result = {}
    for k,v in pairs(t) do
        result[v] = k
    end
    return result
  end