#%%

import cadquery as cq
import cadquery.vis
from cq_warehouse.thread import IsoThread

CASE_X, CASE_Y, CASE_Z = 34.5, 27.6, 55

major_d = 72     # any float
pitch   = 2.5     # any float
thread_height  = 10  #

thread_gap = 0.8  # gap between internal and external threads

cntr_wp = lambda : cq.Workplane("XY")

# External thread solid (by itself, along +Z)
thread = IsoThread(
    major_diameter=major_d - thread_gap,
    pitch=pitch,
    length=thread_height + 3, # give extra length to touch the internal shell
    external=True,
    end_finishes=("fade", "square"),
)

internal_core =    (
        cq.Workplane("XY")
        # .circle(thread.min_radius)
        .circle(thread.min_radius)
        .rect(CASE_X, CASE_Y, centered=True)
        .extrude(thread.length)
    )

# circles at the corners because 3d print makes a weird corner otherwise
internal_core = internal_core.edges(">Z")[1:].vertices().circle(0.5).extrude(-10, combine="cut")


internal_thrd = internal_core.union(thread)

ext_thread = IsoThread(
    major_diameter=major_d,
    pitch=pitch,
    length=thread_height,
    external=False,
    end_finishes=("fade", "square"))

ext_core = cq.Workplane("XY").circle(major_d/2 + 2.5).circle(major_d/2).extrude(thread_height)



external_thrd = ext_core.union(ext_thread)


# cq.vis.show( [internal_thrd, external_thrd])
cq.vis.show( internal_thrd) 
#             cq.vis.style(dbg, color="crimson", linewidth=6))
# cq.vis.show( external_thrd)

cq.exporters.export(internal_thrd, './inner_thread.stl')
# cq.exporters.export(external_thrd, './outer_thread.stl')
#%%


case = cntr_wp().box(CASE_X, CASE_Y, CASE_Z)
d = 72

inner_ball = cntr_wp().sphere(d/2).intersect(
    cntr_wp().box(d*2, d*2, 80)

).intersect(
    cntr_wp().box(100, 100, CASE_Z)
)

#.union(cntr_wp().box(CASE_X,CASE_Y,CASE_Z))


ball = cntr_wp().sphere(d/2 + 4) .cut(
    inner_ball)
    

ball = ball.union(external_thrd.translate((0,0, -thread_height/2)))

# Cut the upper half of the ball (keep lower half)
ball_lower = ball.intersect(
    cq.Workplane("XY")
    .transformed(offset=(0, 0, 0), rotate=(0, 180, 0))
    .box(500, 500, d/2+20, centered=[True, True, False])
)

ball_upper = ball.intersect(
    cq.Workplane("XY")
    .transformed(offset=(0, 0, 0), rotate=(0, 0, 0))
    .box(500, 500, d/2+20, centered=[True, True, False])
)


cq.vis.show(ball_upper, alpha =0.5)

ball_lower = ball_lower.rotate((0, 0, 0), (0, 1, 0), 180)

cq.exporters.export(ball_lower, './ball_lower.stl')
cq.exporters.export(ball_upper, './ball_upper.stl')
# cq.vis.show([inner_ball, case], alpha = 0.5)