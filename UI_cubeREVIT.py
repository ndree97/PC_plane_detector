import math
import pyvista as pv
import vtk

def normalize(v):
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-12:
        return (0.0, 0.0, 1.0)
    return tuple(c / n for c in v)
 
 
def addv(a, b):
    return tuple(x + y for x, y in zip(a, b))
 
 
def mulv(a, s):
    return tuple(x * s for x in a)
 
 
VIEW_MAP = {
    "TOP": ((0, 0, 1), (0, 1, 0)),
    "BOTTOM": ((0, 0, -1), (0, 1, 0)),
    "FRONT": ((0, 1, 0), (0, 0, 1)),
    "BACK": ((0, -1, 0), (0, 0, 1)),
    "RIGHT": ((1, 0, 0), (0, 0, 1)),
    "LEFT": ((-1, 0, 0), (0, 0, 1)),
 
    "TOP_FRONT": ((0, 1, 1), (0, 0, 1)),
    "TOP_BACK": ((0, -1, 1), (0, 0, 1)),
    "TOP_RIGHT": ((1, 0, 1), (0, 0, 1)),
    "TOP_LEFT": ((-1, 0, 1), (0, 0, 1)),
    "BOTTOM_FRONT": ((0, 1, -1), (0, 0, 1)),
    "BOTTOM_BACK": ((0, -1, -1), (0, 0, 1)),
    "BOTTOM_RIGHT": ((1, 0, -1), (0, 0, 1)),
    "BOTTOM_LEFT": ((-1, 0, -1), (0, 0, 1)),
    "FRONT_RIGHT": ((1, 1, 0), (0, 0, 1)),
    "FRONT_LEFT": ((-1, 1, 0), (0, 0, 1)),
    "BACK_RIGHT": ((1, -1, 0), (0, 0, 1)),
    "BACK_LEFT": ((-1, -1, 0), (0, 0, 1)),
 
    "TOP_FRONT_RIGHT": ((1, 1, 1), (0, 0, 1)),
    "TOP_FRONT_LEFT": ((-1, 1, 1), (0, 0, 1)),
    "TOP_BACK_RIGHT": ((1, -1, 1), (0, 0, 1)),
    "TOP_BACK_LEFT": ((-1, -1, 1), (0, 0, 1)),
    "BOTTOM_FRONT_RIGHT": ((1, 1, -1), (0, 0, 1)),
    "BOTTOM_FRONT_LEFT": ((-1, 1, -1), (0, 0, 1)),
    "BOTTOM_BACK_RIGHT": ((1, -1, -1), (0, 0, 1)),
    "BOTTOM_BACK_LEFT": ((-1, -1, -1), (0, 0, 1)),
}
 
 
class RevitLikeViewCube:
    def __init__(self, pl, viewport=(0.80, 0.72, 0.985, 0.985), size=1.0):
        self.pl = pl
        self.size = size
        self.viewport = viewport
        self.ren_main = pl.renderer
        self.ren_win = pl.ren_win
        self.iren = pl.iren.interactor
 
        self.ren_win.SetNumberOfLayers(2)
 
        self.ren_cube = vtk.vtkRenderer()
        self.ren_cube.SetLayer(1)
        self.ren_cube.SetViewport(*viewport)
        self.ren_cube.SetBackground(1, 1, 1)
        self.ren_cube.SetBackgroundAlpha(0.0)
        self.ren_win.AddRenderer(self.ren_cube)
 
        self.actor_to_key = {}
        self.picker = vtk.vtkPropPicker()
 
        self._build_cube()
        self._setup_cube_camera()
        self._bind_events()
 
    def _setup_cube_camera(self):
        cam = self.ren_cube.GetActiveCamera()
        cam.SetPosition(4, -4, 3)
        cam.SetFocalPoint(0, 0, 0)
        cam.SetViewUp(0, 0, 1)
        self.ren_cube.ResetCamera()
        self.ren_cube.ResetCameraClippingRange()
 
    def _bind_events(self):
        self.pl.iren.add_observer("LeftButtonPressEvent", self._on_left_click)
        self.pl.iren.add_observer("RenderEvent", self._on_render)
 
    def _on_render(self, *args):
        self.sync_cube_camera()
 
    def _on_left_click(self, *args):
        x, y = self.iren.GetEventPosition()
        ok = self.picker.Pick(x, y, 0, self.ren_cube)
        if not ok:
            return
 
        actor = self.picker.GetActor()
        key = self.actor_to_key.get(actor)
        if key is None:
            return
 
        self.snap_to_key(key)
        self.ren_win.Render()
 
    def _add_actor(self, mesh, color, opacity=1.0, edge=False):
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(mesh)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetOpacity(opacity)
        if edge:
            actor.GetProperty().SetEdgeVisibility(1)
            actor.GetProperty().SetEdgeColor(0.35, 0.35, 0.35)
            actor.GetProperty().SetLineWidth(1.0)
        self.ren_cube.AddActor(actor)
        return actor
 
    def _basis(self, normal):
        n = normalize(normal)
        helper = (0, 0, 1) if abs(n[2]) < 0.9 else (0, 1, 0)
 
        u = normalize((
            helper[1] * n[2] - helper[2] * n[1],
            helper[2] * n[0] - helper[0] * n[2],
            helper[0] * n[1] - helper[1] * n[0],
        ))
        v = normalize((
            n[1] * u[2] - n[2] * u[1],
            n[2] * u[0] - n[0] * u[2],
            n[0] * u[1] - n[1] * u[0],
        ))
        return u, v, n
 
    def _make_face_label(self, text, center, normal, width):
        txt = pv.Text3D(text, depth=0.001)
        b = txt.bounds
        sx = max(b.x_max - b.x_min, 1e-6)
        sy = max(b.y_max - b.y_min, 1e-6)
        txt = txt.scale((width / max(sx, sy),) * 3, inplace=False)
 
        b = txt.bounds
        c = (
            0.5 * (b.x_min + b.x_max),
            0.5 * (b.y_min + b.y_max),
            0.5 * (b.z_min + b.z_max),
        )
        txt = txt.translate((-c[0], -c[1], -c[2]), inplace=False)
 
        u, v, n = self._basis(normal)
        offset = addv(center, mulv(n, 0.0025))
 
        m = vtk.vtkMatrix4x4()
        vals = [
            [u[0], v[0], n[0], offset[0]],
            [u[1], v[1], n[1], offset[1]],
            [u[2], v[2], n[2], offset[2]],
            [0, 0, 0, 1],
        ]
        for i in range(4):
            for j in range(4):
                m.SetElement(i, j, vals[i][j])
 
        t = vtk.vtkTransform()
        t.SetMatrix(m)
        return txt.transform(t, inplace=False)
 
    def _build_cube(self):
        s = self.size
        hs = s / 2
 
        faces = [
            ("TOP",    (0, 0, hs),  (0, 0, 1),  (0.97, 0.97, 0.97)),
            ("BOTTOM", (0, 0, -hs), (0, 0, -1), (0.84, 0.84, 0.84)),
            ("FRONT",  (0, hs, 0),  (0, 1, 0),  (0.92, 0.92, 0.92)),
            ("BACK",   (0, -hs, 0), (0, -1, 0), (0.84, 0.84, 0.84)),
            ("RIGHT",  (hs, 0, 0),  (1, 0, 0),  (0.90, 0.90, 0.90)),
            ("LEFT",   (-hs, 0, 0), (-1, 0, 0), (0.84, 0.84, 0.84)),
        ]
 
        for key, center, normal, color in faces:
            face = pv.Plane(center=center, direction=normal, i_size=s, j_size=s)
            act = self._add_actor(face, color=color, opacity=1.0, edge=True)
            self.actor_to_key[act] = key
 
            label = self._make_face_label(key, center, normal, width=s * 0.42)
            label_actor = self._add_actor(label, color=(0.30, 0.30, 0.30), opacity=1.0, edge=False)
 
        et = s * 0.16
        el = s * 0.74
        edges = [
            ("TOP_FRONT",    (0, hs, hs),   (el, et, et)),
            ("TOP_BACK",     (0, -hs, hs),  (el, et, et)),
            ("TOP_RIGHT",    (hs, 0, hs),   (et, el, et)),
            ("TOP_LEFT",     (-hs, 0, hs),  (et, el, et)),
            ("BOTTOM_FRONT", (0, hs, -hs),  (el, et, et)),
            ("BOTTOM_BACK",  (0, -hs, -hs), (el, et, et)),
            ("BOTTOM_RIGHT", (hs, 0, -hs),  (et, el, et)),
            ("BOTTOM_LEFT",  (-hs, 0, -hs), (et, el, et)),
            ("FRONT_RIGHT",  (hs, hs, 0),   (et, et, el)),
            ("FRONT_LEFT",   (-hs, hs, 0),  (et, et, el)),
            ("BACK_RIGHT",   (hs, -hs, 0),  (et, et, el)),
            ("BACK_LEFT",    (-hs, -hs, 0), (et, et, el)),
        ]
        for key, center, dims in edges:
            m = pv.Cube(center=center, x_length=dims[0], y_length=dims[1], z_length=dims[2])
            act = self._add_actor(m, color=(1, 0, 0), opacity=0.20, edge=False)
            self.actor_to_key[act] = key
 
        cr = s * 0.10
        corners = [
            ("TOP_FRONT_RIGHT",    (hs, hs, hs)),
            ("TOP_FRONT_LEFT",     (-hs, hs, hs)),
            ("TOP_BACK_RIGHT",     (hs, -hs, hs)),
            ("TOP_BACK_LEFT",      (-hs, -hs, hs)),
            ("BOTTOM_FRONT_RIGHT", (hs, hs, -hs)),
            ("BOTTOM_FRONT_LEFT",  (-hs, hs, -hs)),
            ("BOTTOM_BACK_RIGHT",  (hs, -hs, -hs)),
            ("BOTTOM_BACK_LEFT",   (-hs, -hs, -hs)),
        ]
        for key, center in corners:
            m = pv.Sphere(radius=cr, center=center, theta_resolution=12, phi_resolution=12)
            act = self._add_actor(m, color=(0, 0, 1), opacity=0.20, edge=False)
            self.actor_to_key[act] = key
 
    def scene_center_distance(self):
        b = self.pl.bounds
        center = (
            0.5 * (b.x_min + b.x_max),
            0.5 * (b.y_min + b.y_max),
            0.5 * (b.z_min + b.z_max),
        )
        size = max(b.x_max - b.x_min, b.y_max - b.y_min, b.z_max - b.z_min, 1.0)
        return center, size * 2.6
 
    def snap_to_key(self, key):
        direction, up = VIEW_MAP[key]
        center, dist = self.scene_center_distance()
        pos = addv(center, mulv(normalize(direction), dist))
        self.animate_camera(pos, center, up)
 
    def animate_camera(self, target_pos, target_focal, target_up, steps=16):
        cam = self.ren_main.GetActiveCamera()
        p0 = cam.GetPosition()
        f0 = cam.GetFocalPoint()
        u0 = cam.GetViewUp()
 
        for i in range(1, steps + 1):
            t = i / steps
            s = t * t * (3 - 2 * t)
            pos = tuple((1 - s) * p0[k] + s * target_pos[k] for k in range(3))
            foc = tuple((1 - s) * f0[k] + s * target_focal[k] for k in range(3))
            up = tuple((1 - s) * u0[k] + s * target_up[k] for k in range(3))
            cam.SetPosition(*pos)
            cam.SetFocalPoint(*foc)
            cam.SetViewUp(*up)
            cam.OrthogonalizeViewUp()
            self.ren_main.ResetCameraClippingRange()
            self.sync_cube_camera()
            self.ren_win.Render()
 
    def sync_cube_camera(self):
        cam_main = self.ren_main.GetActiveCamera()
        cam_cube = self.ren_cube.GetActiveCamera()
 
        p = cam_main.GetPosition()
        f = cam_main.GetFocalPoint()
        up = cam_main.GetViewUp()
 
        v = normalize((p[0] - f[0], p[1] - f[1], p[2] - f[2]))
        cam_cube.SetPosition(v[0] * 4.0, v[1] * 4.0, v[2] * 4.0)
        cam_cube.SetFocalPoint(0.0, 0.0, 0.0)
        cam_cube.SetViewUp(*up)
        self.ren_cube.ResetCameraClippingRange()
 
 
def main():
    pl = pv.Plotter(window_size=(1400, 900))
    pl.set_background("white")
 
    mesh = pv.ParametricSuperEllipsoid(n1=0.35, n2=0.8, u_res=100, v_res=100)
    pl.add_mesh(mesh, color="#d9d9d9", smooth_shading=True, specular=0.15)
 
    pl.camera_position = [(5, -5, 4), (0, 0, 0), (0, 0, 1)]
 
    cube = RevitLikeViewCube(pl)
    cube.sync_cube_camera()
 
    pl.show()
 
 
if __name__ == "__main__":
    main()