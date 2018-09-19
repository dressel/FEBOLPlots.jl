__precompile__()


module FEBOLPlots

using FEBOL

using PyPlot: 
    imshow,
    xlabel,
    ylabel,
    contour,
    figure,
    pause,
    hold,
    axis,
    title,
    scatter,
    gcf,
    savefig,
    matplotlib,
    rc,
    tick_params,
    clf,
    cla

using PyPlot
import PyPlot.plot

using Reel      # for gifs

#rc("font", family="serif")
#rc("text", usetex=true)
rc("font", family="Times New Roman", size=16)
println("rc_context = ", rc_context())

export 
    visualize,
    gif,
    plot_vehicle,
    plot,
    hold,
    title,
    pause,
    figure,
    cla,
    gcf,
    savefig

const LocTuple = NTuple{2, Float64}

include("visualize.jl")
include("plotting.jl")
include("gif.jl")
include("special.jl")

end # module
