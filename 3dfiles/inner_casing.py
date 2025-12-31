#%%
import cadquery as cq
import cadquery.vis

INNER_SHELL_W = 2.5

BAT_ELECTRONIC_H = 7.0
BATT_W, BATT_H, BATT_D = 42.0, 29.5, 8.0  # mm

# r = cq.Workplane("XY" ).box(INNER_SHELL_W *2  + BATT_W, 
#                                  INNER_SHELL_W *2 + BATT_H, 
#                                  30)
# v = r.faces(">X").vertices("<Z and <Y")

# r = r.faces(">X").vertices("<Z and <Y").workplane(centerOption="ProjectedOrigin")\
#     .rect(10, 8).cutBlind(-5)  

r = cq.Workplane("XY")\
      .box(INNER_SHELL_W *2  + BATT_W + 5, 
            INNER_SHELL_W *2 + BATT_H, 
            INNER_SHELL_W *2 + BATT_D  + 12)\
      .faces(">X").edges("|Y").edges("<Z").translate((0,0,INNER_SHELL_W))\
      .rect(BATT_W*2, BATT_H).cutBlind(BATT_D)
dbg = None

# dbg = r.edges(">Z and <X")          # vertical edges


# place for esp 
ESP_W, ESP_H = 30.0, 20.5
ESP_W += 4.0  # margin
r = r.edges(">Z and <X")\
      .translate((ESP_W/2 + INNER_SHELL_W, 0,0)).rect(ESP_W, ESP_H, centered=True)\
      .cutBlind(-6)      

dbg = r.edges(">Z and <X")
IMU_W, IMU_H = 10.0, 18.05
r = r.edges(">Z and <X")\
      .translate((INNER_SHELL_W+ + ESP_W+ IMU_W/2, 0, 0)).rect(IMU_W, IMU_H, centered=True)\
      .cutBlind(-12)      


# this are filler holes by extending 
r = r.edges(">Z and <X")\
      .translate(((ESP_W + 20)/2 + INNER_SHELL_W, 0,0)).rect(ESP_W + 20, ESP_H , centered=True)\
      .cutBlind(-6)    
  
r = r.edges(">Z and <X")\
      .translate((INNER_SHELL_W+ 20+ ESP_W+ IMU_W/2, 0, 0)).rect(IMU_W+20, IMU_H, centered=True)\
      .cutBlind(-12)      

# dbg = [r.faces("-Y")[1], r.faces("+Y")[1]]
dbg = r.faces("X")[-3].vertices("<Z and <X and Y")


r = r.faces("X")[-3].vertices("<Z and <X and <Y")\
    .workplane(centerOption="CenterOfMass")\
    .transformed(rotate=(0, 0, 90), offset=(3,-3,0))\
    .rect(6, 6)\
    .extrude(ESP_W, combine="cut")

r = r.faces("X")[-3].edges(">Y").vertices("<Z")\
      .workplane(centerOption="CenterOfMass")\
      .transformed(rotate=(0, 0, 90), offset=(-3,3,0))\
      .rect(6, 6)\
      .extrude(ESP_W, combine="cut")

r = r.faces("-X")[1:].edges("|Z").fillet(0.6)



r = r.edges("Z and >X and <Y").workplane(centerOption="CenterOfMass")\
      .transformed(offset= (7,6,0))\
      .rect(20, 3)\
      .cutBlind(-3)

r = r.edges("X and >Z and <Y")\
      .workplane(centerOption="CenterOfMass")\
      .transformed(offset= (0,0,0), rotate=(0,90,0))\
      .circle(2.5)\
      .cutBlind(-7)


cq.vis.show(
    cq.vis.style(r, alpha=0.5), 
    cq.vis.style(dbg, color="crimson", linewidth=6))  # highlight overlay


# r = r.rotate((0,0,0), (0,1,0), -90)

# cq.exporters.export(r, './inner_box.stl')

#%%

bb = r.val().BoundingBox()

lid_clap = 25

w,h,l = bb.xmax - bb.xmin, bb.ymax - bb.ymin, bb.zmax - bb.zmin

lid = cq.Workplane("XY")\
      .box(w, h, 2.5)


lid = lid.union(
    cq.Workplane("ZY").
      transformed(offset=(-25/2 - 2.5/2, 0, -w/2 - 2.5/2)).
      box(25, h, 2.5)
    )

#perpendicular gace
lid  = lid.union(
    cq.Workplane("ZY").
      transformed(offset=(0, 0, -w/2 - 2.5/2)).
      box(2.5, h, 2.5)
    )

# holder 
lid = lid.union(cq.Workplane("XY")\
                .box(20, 20.8, 6).
      translate((w/2 - 20/2,0,-6/2))
    )


dbg =None

cq.vis.show(
    cq.vis.style(lid, alpha=0.5), 
    cq.vis.style(dbg, color="crimson", linewidth=6))  # highlight overlay


lid = lid.rotate((0,0,0), (0,1,0), 180)

cq.exporters.export(lid, './lid.stl')

# print(f"Bounding box: x=({bb.xmin}, {bb.xmax}), y=({bb.ymin}, {bb.ymax}), z=({bb.zmin}, {bb.zmax})")
#%%)

