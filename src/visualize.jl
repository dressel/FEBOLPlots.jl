#a = rc("font", family="Times New Roman", size=16)
#println("a = ", a)

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

`visualize(m::SearchDomain, su::SimUnit; pause_time=0.3)`
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
function visualize(m::SearchDomain, uav::SimUnit;
                   pause_time=0.3,
                   alpha=0.1,
                   show_mean::Bool=false,
                   save_gif::Bool=true
                  )

    frames = Frames(MIME("image/png"), fps=5)

    #println("rc_context = ", rc_context())
    #rc("font", family="Times New Roman", size=16)
    #rc("text", usetext=true)

    figure("Simulation")
    plot(m, uav.f, uav.x, alpha=alpha)
    #title("i = 0")
    title("\$t\$ = 0 s")
    push!(frames, gcf())

    # What was the cost to getting this first observation?
    cost_sum = get_cost(uav, m)

    # before doing anything else, we observe
    #  and update filter once
    o = observe(m, uav.x)
    update!(uav, o)

    # This was our first step; steps count number of observations
    step_count = 1

    # plot if need be
    plot(m, uav.f, uav.x, alpha=alpha)
    #title("i = $(step_count)")
    title("\$t\$ = $(step_count) s")
    push!(frames, gcf())


    while !is_complete(uav.f, uav.tc, step_count)
        # act
        #tic()
        a = action(m, uav, o)
        #ta = toq()
        #println("time = ", ta)
        act!(m, uav.x, a)

        move_target!(m)

        # get cost and update step count
        cost_val = get_cost(uav, m, a)
        cost_sum += cost_val
        step_count += 1

        # observe and update
        o = observe(m, uav.x)
        update!(uav, o)

        # plot if need be
        pause(pause_time)
        figure("Simulation")
        cla()
        plot(m, uav.f, uav.x, alpha=alpha, show_mean=show_mean)
        #max_b = maximum(uav.f.b)
        #title("i = $(step_count), cost = $(round(cost_val,3))")
        #title("\$t\$ = $(step_count) s, cost = $(round(cost_val,3))")
        title("\$t\$ = $(step_count) s")
        push!(frames, gcf())
    end
    write("temp.mp4", frames)
    write("temp.gif", frames)

    return cost_sum
end

# visualize function for multiple UAVs at once
function visualize(m::SearchDomain, vsu::Vector{SimUnit};
                   pause_time=0.3,
                   alpha=0.2,
                   show_mean::Bool=false
                  )

    #println("rc_context = ", rc_context())
    #rc("font", family="Times New Roman", size=16)
    #rc("text", usetext=true)

    # What was the cost to getting this first observation?
    # TODO: commented out on 8/08/2018, put it back soon
    #cost_sum = get_cost(uav, m)

    # before doing anything else, we observe
    #  and update filter once
    o_vals = []
    for uav in vsu
        o = observe(m, uav.x)
        push!(o_vals, o)
        update!(uav, o)
    end

    # This was our first step; steps count number of observations
    step_count = 1

    # plot if need be
    figure("Simulation")
    plot(m, vsu, alpha=alpha)
    title("i = $(step_count)")


    #while !is_complete(uav.f, uav.tc, step_count)
    while !is_complete(vsu, step_count)
        # act

        for (uav_idx,uav) in enumerate(vsu)
            a = action(m, uav, o_vals[uav_idx])
            act!(m, uav.x, a)
        end

        move_target!(m)

        # get cost and update step count
        # TODO: handle costs for the entire group
        #cost_val = get_cost(uav, m, a)
        #cost_sum += cost_val
        step_count += 1

        # observe and update
        o_vals = []
        for uav in vsu
            o = observe(m, uav.x)
            push!(o_vals, o)
            update!(uav, o)
        end

        # plot if need be
        pause(pause_time)
        figure("Simulation")
        cla()
        plot(m, vsu, alpha=alpha, show_mean=show_mean)
        title("i = $(step_count)")
    end

end
