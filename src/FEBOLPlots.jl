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

import PyPlot.plot

using Reel      # for gifs

rc("font", family="serif")

export 
    visualize,
    gif,
    plot_vehicle,
    plot,
    hold,
    title,
    pause

const LocTuple = NTuple{2, Float64}

include("visualize.jl")
include("plotting.jl")
include("gif.jl")
include("special.jl")

end # module
