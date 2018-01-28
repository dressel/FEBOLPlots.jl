# don't reset the filter, vehicle, and policy
# assume SimUnit comes in clean and ready to go
function visualize(m::SearchDomain, uav::SimUnit; pause_time=0.3)

    # What was the cost to getting this first observation?
    temp_cost = get_cost(uav, m)

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
        temp_cost += get_cost(uav, m, a)
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

    return temp_cost
end
