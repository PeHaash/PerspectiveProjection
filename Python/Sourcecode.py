import ghpythonlib.components as gh
from math import sqrt

if False:
    ACTIVATE = ""

    # two point perspective
    P2_ACTIVATE = ""
    P2_PROJECTION_CENTER = ""
    P2_PROJECTION_WIDTH = ""
    P2_PROJECTION_HEIGHT = ""
    P2_PLANE = ""
    P2_EYE_HEIGHT = ""

    # three point perspective
    P3_ACTIVATE = ""
    P3_GEOMETRIC_POINTS = ""
    P3_PROJECTED_POINTS = ""
    P3_PERSPECTIVE_FACTORS = ""

    # fish eye perspective
    FE_ACTIVATE = ""
    FE_EYE = ""
    FE_TARGET = ""
    FE_DESTINATION_CENTER = ""
    FE_DESTINATION_RADIUS = ""

    # projection variables
    MESH_DEFAULT_COLOR = ""
    TOLERANCE = ""

    # input data
    POINTS = ""
    POLYLINES = ""
    MESHES = ""

    # output data
    OUTPUT_POINTS = ""
    OUTPUT_POLYLINES = ""
    OUTPUT_MESHES = ""


def Flatten(list_of_lists):
    if len(list_of_lists) == 0:
        return list_of_lists
    if isinstance(list_of_lists[0], list):
        return Flatten(list_of_lists[0]) + Flatten(list_of_lists[1:])
    return list_of_lists[:1] + Flatten(list_of_lists[1:])


class point_temp(object):
    def __init__(self, x, y):
        self.X = x
        self.Y = y


def Middle(p1, p2, t2=1, t1=1):
    return point_temp((p1.X * t1 + p2.X * t2) / (t1 + t2), (p1.Y * t1 + p2.Y * t2) / (t1 + t2))


def Intersection4Points(p1, p2, p3, p4):  # point in intersections of line(p1,p2)--line(p3,p4)
    point = point_temp(0, 0)
    dist = (p1.X - p2.X) * (p3.Y - p4.Y) - (p1.Y - p2.Y) * (p3.X - p4.X)
    point.X = ((p1.X * p2.Y - p1.Y * p2.X) * (p3.X - p4.X) - (p1.X - p2.X) * (p3.X * p4.Y - p3.Y * p4.X)) / dist
    point.Y = ((p1.X * p2.Y - p1.Y * p2.X) * (p3.Y - p4.Y) - (p1.Y - p2.Y) * (p3.X * p4.Y - p3.Y * p4.X)) / dist
    return point


class TwoPointPerspective:
    def __init__(self, plane, eye_height, p_center, p_width, p_height):

        if eye_height == 0:
            raise ArithmeticError
        self.Plane = plane
        self.EyeHeight = eye_height
        self.HorizonY = p_center.Y
        horizon_vector = gh.VectorXYZ(p_width, 0, 0)[0]
        n_sign = (-1 if eye_height > 0 else 1)
        self.PointO = gh.Move(p_center, gh.VectorXYZ(0, n_sign * p_height, 0)[0])[0]
        self.vpLeft = gh.Move(p_center, -horizon_vector)[0]
        self.vpRight = gh.Move(p_center, horizon_vector)[0]
        self.horizon_line = gh.Line(self.vpRight, self.vpLeft)

        point_1 = gh.Move(p_center, gh.VectorXYZ(-1, -n_sign * 2, 0)[0])[0]
        point_2 = gh.Move(p_center, gh.VectorXYZ(1, -n_sign * 2, 0)[0])[0]

        point_r = Intersection4Points(self.PointO, self.vpRight, p_center, point_1)
        point_l = Intersection4Points(self.PointO, self.vpLeft, p_center, point_2)

        self.ModuleLenght = abs(eye_height) / sqrt(2)
        self.PointL = gh.ConstructPoint(point_l.X, point_l.Y, 0)
        self.PointR = gh.ConstructPoint(point_r.X, point_r.Y, 0)

    def Project(self, point):
        X, Y, Z = gh.PlaneCoordinates(point, self.Plane)
        x = X / self.ModuleLenght
        y = Y / self.ModuleLenght
        x_on_module = Middle(self.PointL, self.PointR, x, (1 - x))
        y_on_module = Middle(self.PointR, self.PointL, y, (1 - y))
        base = Intersection4Points(self.vpRight, y_on_module, self.vpLeft, x_on_module)
        y_final = (Z * (self.HorizonY - base.Y) / self.EyeHeight) + base.Y
        return gh.ConstructPoint(base.X, y_final, -(X * X + Y * Y + Z * Z))


class ThreePointPerspective:
    def __init__(self, geometric_points, projected_points, perspective_factors):
        geo_o, geo_x, geo_y, geo_z = geometric_points
        pro_o, pro_x, pro_y, pro_z = projected_points
        fact_x, fact_y, fact_z = perspective_factors

        v_x = gh.Vector2Pt(geo_o, geo_x, False)[0]
        v_y = gh.Vector2Pt(geo_o, geo_y, False)[0]
        v_z = gh.Vector2Pt(geo_o, geo_z, False)[0]

        matrix = gh.ConstructMatrix(3, 3, [v_x.X, v_y.X, v_z.X,
                                           v_x.Y, v_y.Y, v_z.Y,
                                           v_x.Z, v_y.Z, v_z.Z])

        inverted_matrix, is_success = gh.InvertMatrix(matrix, 0)
        if not is_success:
            raise ArithmeticError

        self.Matrix = inverted_matrix
        self.PointO = geo_o
        self.BoxO, self.BoxX, self.BoxY, self.BoxZ = pro_o, pro_x, pro_y, pro_z

        self.PersPoX = gh.Move(pro_o, gh.Vector2Pt(pro_o, pro_x, False)[0] * fact_x)[0]
        self.PersPoY = gh.Move(pro_o, gh.Vector2Pt(pro_o, pro_y, False)[0] * fact_y)[0]
        self.PersPoZ = gh.Move(pro_o, gh.Vector2Pt(pro_o, pro_z, False)[0] * fact_z)[0]

    def Project(self, point):
        position = gh.Vector2Pt(self.PointO, point, False)[0]
        X, Y, Z = gh.DeconstructMatrix(gh.Multiplication(self.Matrix, position))[2]
        x_on_triangle = Middle(self.BoxY, self.BoxX, X, (1 - X))
        y_on_triangle = Middle(self.BoxZ, self.BoxY, Y, (1 - Y))
        z_on_triangle = Middle(self.BoxX, self.BoxZ, Z, (1 - Z))
        x_on_axis = Intersection4Points(self.BoxO, self.PersPoX, self.PersPoY, x_on_triangle)
        y_on_axis = Intersection4Points(self.BoxO, self.PersPoY, self.PersPoZ, y_on_triangle)
        z_on_axis = Intersection4Points(self.BoxO, self.PersPoZ, self.PersPoX, z_on_triangle)
        intersec1 = Intersection4Points(self.PersPoZ, x_on_axis, self.PersPoX, z_on_axis)
        intersec2 = Intersection4Points(self.PersPoZ, y_on_axis, self.PersPoY, z_on_axis)
        projected_point = Intersection4Points(intersec1, self.PersPoY, intersec2, self.PersPoX)
        return gh.ConstructPoint(projected_point.X, projected_point.Y, -(X * X + Y * Y + Z * Z))


class FishEyePerspective:
    def __init__(self, eye, target, destination_center, destination_radius):
        self.X0, self.Y0, self.Z0 = gh.Deconstruct(destination_center)
        self.Radius = destination_radius
        self.Plane = self.MakePlane(eye, target)

    def MakePlane(self, eye, target):
        vector_1 = gh.Vector2Pt(eye, target, True)[0]
        if vector_1.X == 0 and vector_1.Y == 0:
            # is looking at sky
            plane_temp = gh.XYPlane(eye)
            if eye.Z < target.Z:
                plane_temp = gh.FlipPlane(plane_temp, False, False, True)
            return plane_temp  # -gh.XYPlane(eye)  # if eye.Z < target.Z else gh.XYPlane(eye)
        vector_2 = gh.CrossProduct(vector_1, gh.UnitZ(1), False)[0]
        plane_temp = gh.ConstructPlane(eye, vector_1, vector_2)
        plane_temp = gh.AdjustPlane(plane_temp, vector_1)
        plane_final = gh.FlipPlane(plane_temp, False, False, True)
        return plane_final

    def Project(self, point):
        x, y, z = gh.PlaneCoordinates(point, self.Plane)
        dis = sqrt(x * x + y * y + z * z)
        proj_x = x / dis * self.Radius
        proj_y = y / dis * self.Radius
        if z > 0:
            dist_to_cent = sqrt(proj_x * proj_x + proj_y * proj_y)
            return gh.ConstructPoint(self.X0 + proj_x * self.Radius / dist_to_cent,
                                     self.Y0 + proj_y * self.Radius / dist_to_cent, self.Z0 - dis)
        return gh.ConstructPoint(self.X0 + proj_x, self.Y0 + proj_y, self.Z0 - dis)


def ProjectSegment(segment, perspective_system, tolerance):
    open_or_closed = gh.Closed(segment)[0]
    divide_number = gh.Length(segment) // tolerance + 1
    points = gh.DivideCurve(segment, divide_number, False)[0]
    return gh.PolyLine([perspective_system.Project(point) for point in points], open_or_closed)


def ProjectPolyline(polyline, perspective_system, tolerance):
    segments = gh.Explode(polyline, True)[0]
    if not isinstance(segments, list):
        segments = [segments]
    return (gh.JoinCurves([ProjectSegment(seg, perspective_system, tolerance) for seg in segments], False))


def ProjectMesh(mesh, perspective_system, default_color):
    vertices, faces, colors, normals = gh.DeconstructMesh(mesh)
    if colors is None:
        colors = default_color
    return gh.ConstructMesh([perspective_system.Project(vertex) for vertex in vertices], faces, colors)


if ACTIVATE:
    perspective_systems = []
    if P2_ACTIVATE:
        try:
            pers2p = TwoPointPerspective(P2_PLANE, P2_EYE_HEIGHT, P2_PROJECTION_CENTER, P2_PROJECTION_WIDTH, P2_PROJECTION_HEIGHT)
            perspective_systems.append(pers2p)
        except ArithmeticError:
            print("Error: P2_EYE_HEIGHT cannot be zero")
    if P3_ACTIVATE:
        try:
            pers3p = ThreePointPerspective(P3_GEOMETRIC_POINTS, P3_PROJECTED_POINTS, P3_PERSPECTIVE_FACTORS)
            perspective_systems.append(pers3p)
        except ArithmeticError:
            print("Error: Cannot Invert the Matrix: problem with P3_GEOMETRIC_POINTS")
    if FE_ACTIVATE:
        pers5p = FishEyePerspective(FE_EYE, FE_TARGET, FE_DESTINATION_CENTER, FE_DESTINATION_RADIUS)
        perspective_systems.append(pers5p)

    OUTPUT_POINTS = []
    OUTPUT_POLYLINES = []
    OUTPUT_MESHES = []
    for system in perspective_systems:
        OUTPUT_POINTS.extend([system.Project(point) for point in POINTS])
        OUTPUT_POLYLINES.extend(Flatten([ProjectPolyline(polyline, system, TOLERANCE) for polyline in POLYLINES]))
        OUTPUT_MESHES.extend([ProjectMesh(mesh, system, MESH_DEFAULT_COLOR) for mesh in MESHES])
