"""
Plots each step of a simulation.

`visualize(m, x, f, p, n_steps=10; pause_time=0.3)`

where
* `m` is a `SearchDomain`
* `x` is a `Vehicle`
* `f` is a subtype of `AbstractFilter`
* `p` is a subtype of `Policy`
* `n_steps` is the number of steps to simulate
* `pause_time` is the time to wait between steps

An alternative call uses the `SimUnit` type:

`visualize(m::SearchDomain, su::SimUnit; pause_time=0.3`)
"""
function visualize(m::SearchDomain,
                   x::Vehicle,
                   f::AbstractFilter,
                   p::Policy,
                   n_steps::Int=10;
                   pause_time::Real = 0.3
                  )

    tc = StepThreshold(n_steps)
    su = SimUnit(x, f, p, tc)
    visualize(m, su; pause_time=pause_time)
end

# doesn't reset the filter, vehicle, and policy
# assume SimUnit comes in clean and ready to go
function visualize(m::SearchDomain, uav::SimUnit; pause_time=0.3)

    # What was the cost to getting this first observation?
    cost_sum = get_cost(uav, m)

    # before doing anything else, we observe
    #  and update filter once
    o = observe(m, uav.x)
    update!(uav, o)

    # This was our first step; steps count number of observations
    step_count = 1

    # plot if need be
    figure("Simulation")
    plot(m, uav.f, uav.x)
    title("i = $(step_count)")


    while !is_complete(uav.f, uav.tc, step_count)
        # act
        a = action(m, uav, o)
        act!(m, uav.x, a)

        # get cost and update step count
        cost_sum += get_cost(uav, m, a)
        step_count += 1

        # observe and update
        o = observe(m, uav.x)
        update!(uav, o)

        # plot if need be
        pause(pause_time)
        figure("Simulation")
        cla()
        plot(m, uav.f, uav.x)
        max_b = maximum(uav.f.b)
        title("i = $(step_count), max = $(round(max_b,3))")
    end

    return cost_sum
end
