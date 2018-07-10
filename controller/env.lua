local controller = {}
function controller.determinant(x)
    -- calculate determinant of matrix using Gaussian elimination
    -- see https://en.wikipedia.org/wiki/Gaussian_elimination
    assert(x:dim() == 2 and x:size(1) == x:size(2), 'Square matrix expected')
    x = x:clone()
    local swaps = 0
    for i = 1, x:size(1) do
        -- choose pivot with largest absolute value, handles x[i,i] == 0 case & improve numerical stability
        local p, p_ind = torch.max(torch.abs(x[{{i, x:size(1)}, i}]), 1)
        local max_i = p_ind[1] + i - 1 -- index of pivot
        local max_v = x[{max_i, i}] -- value of pivot
        if p[1] > 1e-8 then -- skip column if no pivot ~= 0 is available
            -- swap rows
            if max_i ~= i then
                local t = x[i]:clone()
                x[i] = x[max_i]
                x[max_i] = t
                swaps = swaps + 1
            end
            for j = i + 1, x:size(1) do
                x[j]:add(-(x[{j, i}] / max_v), x[i])
            end
        end
    end
    local d = x:diag():prod()
    -- negate determinant when we swapped rows an odd number of times
    -- (swapping two rows multiplies determinant by -1)
    if swaps % 2 ~= 0 then
        d = -d
    end
    return d
end

--- calculates the weighted pseudoInverse of M
-- @param M: Matrix which needs to be inversed
-- @param W: weight Matrix. (Optional)
function controller.pseudoInverse(M, W)
    local weights = W or torch.eye(M:size(1))
    assert(M:size(1) == weights:size()[1], 'Data matrix M and weight matrix W need to have the same number of cols')
    local inv = M:t() * weights * M
    -- make it definite
    inv:add(1e-15, torch.eye(inv:size(1)))
    return torch.inverse(inv) * M:t() * weights
end

function controller.detectNan(vector)
    for i = 1, vector:size(1) do
        if vector[i] ~= vector[i] then
            return true
        end
    end
    return false
end

return controller
