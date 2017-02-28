f = '/home/pat/Desktop/blue_funnel.obj'

meshes, actors = ioUtils.readObjMtl(f)

for m, a in zip(meshes, actors):

    color = a.GetProperty().GetColor()
    opacity = a.GetProperty().GetOpacity()

    obj = vis.showPolyData(m, 'mesh', color=color)
    obj.setProperty('Alpha', opacity)
    obj.actor.SetTexture(a.GetTexture())
