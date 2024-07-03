using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino;
using Rhino.Geometry;

namespace PeHash
{
    public abstract class PerspectiveProjection
    {
        protected static Point3d Intersection4Points(Point3d p1, Point3d p2, Point3d p3, Point3d p4)
        {
            // point in intersections of line(p1,p2)--line(p3,p4)
            double dist = (p1.X - p2.X) * (p3.Y - p4.Y) - (p1.Y - p2.Y) * (p3.X - p4.X);
            return new Point3d(
                ((p1.X * p2.Y - p1.Y * p2.X) * (p3.X - p4.X) - (p1.X - p2.X) * (p3.X * p4.Y - p3.Y * p4.X)) / dist,
                ((p1.X * p2.Y - p1.Y * p2.X) * (p3.Y - p4.Y) - (p1.Y - p2.Y) * (p3.X * p4.Y - p3.Y * p4.X)) / dist,
                0);
        }

        protected static Point3d Middle(Point3d p1, Point3d p2, double t2 = 1, double t1 = 1)
        {
            return new Point3d((p1.X * t1 + p2.X * t2) / (t1 + t2), (p1.Y * t1 + p2.Y * t2) / (t1 + t2), 0);
        }

        public Polyline[] ProjectCurves(Curve[] curves, double Tolerance)
        {
            List<Polyline> ret = new List<Polyline>();
            foreach (Curve curve in curves)
            {
                Curve[] exploded_curves = curve.DuplicateSegments();
                foreach (Curve segment in exploded_curves)
                {
                    List<Point3d> points_in_segment = new List<Point3d>();
                    Point3d[] pnts;
                    points_in_segment.Add(Project(segment.PointAtStart));
                    segment.DivideByLength(Tolerance, false, out pnts);
                    if (pnts != null)
                    {
                        points_in_segment.AddRange(pnts.Select(x => Project(x)));
                    }
                    points_in_segment.Add(Project(segment.PointAtEnd));
                    ret.Add(new Polyline(points_in_segment));
                }
            }
            return ret.ToArray();
        }

        public Mesh ProjectMesh(Mesh mesh)
        {
            Mesh ret = mesh.DuplicateMesh();
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                ret.Vertices.SetVertex(i, Project(mesh.Vertices[i]));
            }
            ret.Normals.ComputeNormals();
            ret.Compact();
            return ret;
        }

        public abstract Point3d Project(Point3d point);

    }
    
    public class TwoPoints: PerspectiveProjection
    {
        Plane mPlane;
        double EyeHeight, HorizonY ,ModuleLenght;
        Point3d Point0, vpLeft, vpRight, PointL, PointR;
        public TwoPoints(Plane plane, double eye_height, Point3d p_center, double p_width, double p_height) { 
            if (eye_height == 0)
            {
                throw new ArithmeticException(); // eye height can not be zero
            }
            mPlane = plane;
            EyeHeight = eye_height;
            HorizonY = p_center.Y;
            Vector3d horizon_vector = new Vector3d(p_width, 0, 0);
            int n_sign = eye_height > 0 ? -1 : 1;
            Point0 = vpLeft = vpRight = p_center;
            Point0.Transform(Transform.Translation(0, n_sign * p_height, 0));
            vpLeft.Transform(Transform.Translation(-horizon_vector));
            vpRight.Transform(Transform.Translation(horizon_vector));

            Point3d point_1 = p_center, point_2 = p_center;
            point_1.Transform(Transform.Translation(-1, -n_sign * 2, 0));
            point_2.Transform(Transform.Translation(1, -n_sign * 2, 0));

            PointL = Intersection4Points(Point0, vpLeft, p_center, point_2);
            PointR = Intersection4Points(Point0, vpRight, p_center, point_1);
            ModuleLenght = Math.Abs(eye_height) / Math.Sqrt(2);

        }
        public override Point3d Project(Point3d point)
        {
            Point3d pt_out;
            mPlane.RemapToPlaneSpace(point, out pt_out);
            double x = pt_out.X / ModuleLenght;
            double y = pt_out.Y / ModuleLenght;
            Point3d x_on_module = Middle(PointL, PointR, x, 1 - x);
            Point3d y_on_module = Middle(PointR, PointL, y, 1 - y);
            Point3d Base = Intersection4Points(vpRight, y_on_module, vpLeft, x_on_module);
            double y_final = (pt_out.Z * (HorizonY - Base.Y) / EyeHeight) + Base.Y;
            return new Point3d(Base.X, y_final, -(pt_out.X * pt_out.X + pt_out.Y * pt_out.Y + pt_out.Z * pt_out.Z));
        }

    }

    public class ThreePoints : PerspectiveProjection
    {
        Matrix mMatrix;
        Point3d Point0, BoxO, BoxX, BoxY, BoxZ;
        Point3d PersPoX, PersPoY, PersPoZ;


        public ThreePoints(Point3d[] geometric_points, Point3d[] projected_points, double[] perspective_factors) {
            Point3d geo_o = geometric_points[0];
            Point3d geo_x = geometric_points[1];
            Point3d geo_y = geometric_points[2];
            Point3d geo_z = geometric_points[3];

            Point3d pro_o = projected_points[0];
            Point3d pro_x = projected_points[1];
            Point3d pro_y = projected_points[2];
            Point3d pro_z = projected_points[3];

            double fact_x = perspective_factors[0];
            double fact_y = perspective_factors[1];
            double fact_z = perspective_factors[2];

            Vector3d v_x = geo_x - geo_o, v_y = geo_y - geo_o, v_z = geo_z - geo_o;
            mMatrix = new Matrix(3, 3);
            mMatrix[0, 0] = v_x.X;
            mMatrix[0, 1] = v_y.X;
            mMatrix[0, 2] = v_z.X;
            mMatrix[1, 0] = v_x.Y;
            mMatrix[1, 1] = v_y.Y;
            mMatrix[1, 2] = v_z.Y;
            mMatrix[2, 0] = v_x.Z;
            mMatrix[2, 1] = v_y.Z;
            mMatrix[2, 2] = v_z.Z;

            bool is_success = mMatrix.Invert(0);
            if (!is_success)
            {
                throw new ArithmeticException(); // non-invertable matrix
            }

            Point0 = geo_o;
            BoxO = pro_o; BoxX = pro_x; BoxY = pro_y; BoxZ = pro_z;

            PersPoX = PersPoY = PersPoZ = pro_o;
            PersPoX.Transform(Transform.Translation((pro_x - pro_o) * fact_x));
            PersPoY.Transform(Transform.Translation((pro_y - pro_o) * fact_y));
            PersPoZ.Transform(Transform.Translation((pro_z - pro_o) * fact_z));
        }

        public override Point3d Project(Point3d point)
        {
            Matrix pos = new Matrix(3, 1);
            pos[0, 0] = point.X - Point0.X;
            pos[1, 0] = point.Y - Point0.Y;
            pos[2, 0] = point.Z - Point0.Z;
            Matrix xyz = mMatrix * pos;
            double X = xyz[0, 0], Y = xyz[1, 0], Z = xyz[2, 0];
            Point3d x_on_triangle = Middle(BoxY, BoxX, X, 1 - X);
            Point3d y_on_triangle = Middle(BoxZ, BoxY, Y, 1 - Y);
            Point3d z_on_triangle = Middle(BoxX, BoxZ, Z, 1 - Z);

            Point3d x_on_axis = Intersection4Points(BoxO, PersPoX, PersPoY, x_on_triangle);
            Point3d y_on_axis = Intersection4Points(BoxO, PersPoY, PersPoZ, y_on_triangle);
            Point3d z_on_axis = Intersection4Points(BoxO, PersPoZ, PersPoX, z_on_triangle);

            Point3d intersec1 = Intersection4Points(PersPoZ, x_on_axis, PersPoX, z_on_axis);
            Point3d intersec2 = Intersection4Points(PersPoZ, y_on_axis, PersPoY, z_on_axis);
            Point3d projected_point = Intersection4Points(intersec1, PersPoY, intersec2, PersPoX);
            return new Point3d(projected_point.X, projected_point.Y, -(X * X + Y * Y + Z * Z));

        }


    }

    public class FishEye : PerspectiveProjection
    {
        double X0, Y0, Z0, Radius;
        Plane mPlane;
        public FishEye(Point3d eye, Point3d target, Point3d destination_center, double destination_radius)
        {
            X0 = destination_center.X; Y0 = destination_center.Y; Z0 = destination_center.Z;
            Radius = destination_radius;
            mPlane = MakePlane(eye, target);
        }

        private static Plane AdjustPlane(Plane originalPlane, Vector3d targetNormal)
        {
            // gh.AdjustPlane rewritten in C#, courtesy of chatGPT

            // Ensure the targetNormal is a unit vector
            targetNormal.Unitize();

            // Create a rotation transformation from the original plane's normal to the target normal
            Vector3d originalNormal = originalPlane.ZAxis;
            Transform rotation = Transform.Rotation(originalNormal, targetNormal, originalPlane.Origin);

            // Apply the rotation to the original plane
            Plane adjustedPlane = originalPlane;
            adjustedPlane.Transform(rotation);

            return adjustedPlane;
        }

        private static Plane MakePlane(Point3d eye, Point3d target)
        {
            Vector3d vector_1 = target - eye;
            if (vector_1.X == 0 && vector_1.Y == 0)
            {
                if (eye.Z > target.Z)
                {
                    // is looking at sky
                    return new Plane(eye, new Vector3d(0, 0, 1));
                }
                else
                {
                    // looking at floor
                    return new Plane(eye, new Vector3d(0, 0, -1));
                }
            }
            Vector3d vector_2 = Vector3d.CrossProduct(vector_1, Vector3d.ZAxis);
            Plane temp_plane = AdjustPlane(new Plane(eye, vector_1, vector_2), vector_1);
            temp_plane.Flip();
            return temp_plane;
        }
        public override Point3d Project(Point3d point)
        {
            Point3d pt_out;
            mPlane.RemapToPlaneSpace(point, out pt_out);
            double dis = ((Vector3d) pt_out).Length;
            double proj_x = pt_out.X / dis * Radius;
            double proj_y = pt_out.Y / dis * Radius;
            if (pt_out.Z > 0)
            {
                double dist_to_cent = Math.Sqrt(proj_x * proj_x + proj_y * proj_y);
                return new Point3d(X0 + proj_x * Radius / dist_to_cent,
                  Y0 + proj_y * Radius / dist_to_cent, Z0 - dis);
            }
            return new Point3d(X0 + proj_x, Y0 + proj_y, Z0 - dis);
        }


    }


}
