
using System.Diagnostics.SymbolStore;
using System.Dynamic;
using System.IO.Compression;
using System.Numerics;
using System.Runtime.ConstrainedExecution;

string pixelBrightnesses = " .'`^\",:;Il!i><~+_-?][}{1)(|\\/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$";
int cameraWidth = 120;
int fov = 90;
float viewDistance = 100;
float sceneBrightness = 8;
Ray cameraCenter = new Ray(Vector3.Zero, new Vector3(0, 0, -1));
Camera camera = new Camera(cameraCenter, fov, cameraWidth, viewDistance);
List<Vertex> allVertices = new List<Vertex>();




Object cube = Primitives.CreateCube(allVertices, 2, new Vector3(0,0,-4), new Vector3(MathF.PI/4,MathF.PI/4, MathF.PI/2));
List<Object> objects = new List<Object>();
List<Vector3> movements = new List<Vector3>();
List<Vector3> rotations = new List<Vector3>();
List<Vector3> rotationPoints = new List<Vector3>();
objects.Add(cube);
movements.Add(new Vector3(0,0,0));
rotations.Add(new Vector3(0,0.1f,0f));
rotationPoints.Add(objects[0].Center);


while (true)
{
    Console.WriteLine("(A)nimate, (C)reate primitive, (D)estroy Primative, (S)et movements, (I)nfo");
    ConsoleKey input = Console.ReadKey().Key;
    Console.WriteLine();
    if (input == ConsoleKey.A)
    {
        Console.WriteLine("How many frames:");
        string frameInput = Console.ReadLine();
        int frames;
        if (int.TryParse(frameInput, out frames))
        {
            DrawFrames(frames);
        }
        else
        {
            Console.WriteLine("Not an integer, try again.");
        }
        
        
    }
    else if (input == ConsoleKey.C)
    {
        Console.WriteLine("Which primitive? (Cube)");
        string primitiveInput = Console.ReadLine();
        if (primitiveInput == "cube" || primitiveInput == "Cube")
        {
            Console.WriteLine("Side length:");
            float sideLength = float.Parse(Console.ReadLine());
            Console.WriteLine("Position X:");
            Vector3 position = new Vector3(float.Parse(Console.ReadLine()), 0 ,0);
            Console.WriteLine("Position Y:");
            position = new Vector3(position.X, float.Parse(Console.ReadLine()) ,0);
            Console.WriteLine("Position Z:");
            position = position with { Z = float.Parse(Console.ReadLine()) };
            
            Console.WriteLine("Rotation X:");
            Vector3 rotation = new Vector3(float.Parse(Console.ReadLine()), 0 ,0);
            Console.WriteLine("Rotation Y:");
            rotation = new Vector3(rotation.X, float.Parse(Console.ReadLine()) ,0);
            Console.WriteLine("Rotation Z:");
            rotation = position with { Z = float.Parse(Console.ReadLine()) };
            
            
            
            objects.Add(Primitives.CreateCube(allVertices, sideLength, position, rotation));
            rotations.Add(Vector3.Zero);
            rotationPoints.Add(Vector3.Zero);
            movements.Add(Vector3.Zero);
        }
        


    }
    else if (input == ConsoleKey.S)
    {
        WriteAllPrimitives();
        
        Console.WriteLine("\n Input the i of the object to change");
        int objectInput = Int32.Parse(Console.ReadLine()) - 1;

        Console.WriteLine("i  Center:                     ");
        Console.WriteLine((objectInput + 1) + " " + objects[objectInput].Center);
        Console.WriteLine("Movement X per frame: ");
        movements[objectInput] = movements[objectInput] with { X = float.Parse(Console.ReadLine()) };
        Console.WriteLine("Movement Y per frame: ");
        movements[objectInput] = movements[objectInput] with { Y = float.Parse(Console.ReadLine()) };
        Console.WriteLine("Movement Z per frame: ");
        movements[objectInput] = movements[objectInput] with { Z = float.Parse(Console.ReadLine()) };
        Console.WriteLine("Rotation X per frame: ");
        rotations[objectInput] = rotations[objectInput] with { X = float.Parse(Console.ReadLine()) };
        Console.WriteLine("Rotation Y per frame: ");
        rotations[objectInput] = rotations[objectInput] with { Y = float.Parse(Console.ReadLine()) };
        Console.WriteLine("Rotation Z per frame: ");
        rotations[objectInput] = rotations[objectInput] with { Z = float.Parse(Console.ReadLine()) };
        Console.WriteLine("Rotation Around Point X");
        rotationPoints[objectInput] = rotationPoints[objectInput] with { X = float.Parse(Console.ReadLine()) };
        Console.WriteLine("Rotation Around Point Y");
        rotationPoints[objectInput] = rotationPoints[objectInput] with { Y = float.Parse(Console.ReadLine()) };
        Console.WriteLine("Rotation Around Point Z");
        rotationPoints[objectInput] = rotationPoints[objectInput] with { Z = float.Parse(Console.ReadLine()) };
        
        
        
    }
    else if(input == ConsoleKey.D)
    {
        WriteAllPrimitives();
        Console.WriteLine("\n Input the i of the object to destroy");
        int objectInput = Int32.Parse(Console.ReadLine()) - 1;
        Object primitive = objects[objectInput];
        foreach (var vertex in primitive.Vertices)
        {
            allVertices.Remove(vertex);
        }
        movements.RemoveAt(objectInput);
        rotations.RemoveAt(objectInput);
        rotationPoints.RemoveAt(objectInput);
        objects.RemoveAt(objectInput);

    }
    else if (input == ConsoleKey.I)
    {
        Console.WriteLine("This is a basic 3D rendering software, built entirely for a console application. ");
        Console.WriteLine("Made by Joel Harrison, ");
    }
    
}



void WriteAllPrimitives()
{
    Console.WriteLine("i  Center:                     Rotation Per Second:                  Movement Per Second:");
    for (int i = 1; i < objects.Count + 1; i++)
    {
        Console.WriteLine();
        Console.Write(i + " " + objects[i - 1].Center + " " + rotations[i-1] + " " + movements[i-1]);
    }
}

void DrawFrames(int frames)
{
    for (int frameNum = 0; frameNum < frames; frameNum++)
    {
        float[,] frame = FrameBrightnessData(camera);
        Console.Clear();
        for (int i = 0; i < frame.GetLength(0); i++)
        {
            for (int j = 0; j < frame.GetLength(1); j++)
            {
                int pixelBrightness = (int)MathF.Pow(frame[i, j] * sceneBrightness, 2);
                if(pixelBrightness < pixelBrightnesses.Length)
                    Console.Write(pixelBrightnesses[pixelBrightness]);
                else
                {
                    Console.Write(pixelBrightnesses[^1]);
                }
                
        
            }
            Console.WriteLine();
        }
        
        Thread.Sleep(10);
        
        for (int i = 0; i < objects.Count; i++)
        {
            objects[i].Rotate(rotationPoints[i], rotations[i]);
            objects[i].Move(movements[i]);
        }


    }
    
}

float[,] FrameBrightnessData(Camera camera)
{
    float[,] final = new float[camera.CameraWidth, camera.CameraWidth];
    int finalIndexX = 0;
    int finalIndexY = 0;
    for (var rayIndex = 0; rayIndex < camera.AllCameraRays.Count; rayIndex++)
    {
        
        var pixel = camera.AllCameraRays[rayIndex];
        float closestVertexDistance = camera.ViewDistance + 1;
        Vertex closestVertex;
        for (var vertexIndex = 0; vertexIndex < allVertices.Count; vertexIndex++)
        {
            var vertex = allVertices[vertexIndex];
            Vector3 intersection = pixel.Intersection(vertex);
            if (VectorMath.PointInVertex(intersection, vertex) && closestVertexDistance > pixel.Distance(vertex))
            {
                closestVertex = vertex;
                closestVertexDistance = pixel.Distance(vertex);
            }
        }

        if (1 / closestVertexDistance > 1 / viewDistance)
            final[finalIndexX, finalIndexY] = 1 / closestVertexDistance;
        else
            final[finalIndexX, finalIndexY] = 0;
        finalIndexX++;
        if (finalIndexX == cameraWidth)
        {
            finalIndexX = 0;
            finalIndexY++;
        }

    }

    return final;

}


class Vertex
{
    public Vector3 Point1;
    public Vector3 Point2;
    public Vector3 Point3;
    
    public Vertex(Vector3 point1, Vector3 point2, Vector3 point3)
    {
        Point1 = point1;
        Point2 = point2;
        Point3 = point3;
    }

    public void Rotate(Vector3 point, Vector3 rotation)
    {
        Move(-point);

        Point1 = VectorMath.RotateVector(Point1, rotation);
        Point2 = VectorMath.RotateVector(Point2, rotation);
        Point3 = VectorMath.RotateVector(Point3, rotation);

        Move(point);
    }

    public void Move(Vector3 movement)
    {
        Point1 += movement;
        Point2 += movement;
        Point3 += movement;
    }

    public void Scale(float scalar)
    {
        Point1 *= scalar;
        Point2 *= scalar;
        Point3 *= scalar;
    }
    
}

class Ray
{
    public Vector3 Origin;
    public Vector3 Direction;

    public Ray(Vector3 origin, Vector3 direction)
    {
        Origin = origin;
        Direction = direction;
    }
    public Vector3 Intersection(Vertex vertex)
    {
        Plane plane = VectorMath.GetPlane(vertex);
        float dotProduct = Vector3.Dot(Direction, plane.Normal);
        if (dotProduct >= 0.00000001)
        {
            float t = (Vector3.Dot((vertex.Point1 - Vector3.Zero), plane.Normal)) / dotProduct;
            return t * Direction;
            
        }

        return new Vector3(10000000,10000000,10000000);
    }

    public float Distance(Vertex vertex)
    {
        return (Origin + Intersection(vertex)).Length();
    }

}

class Object
{
    public Vertex[] Vertices;
    public Vector3 Center;

    public Object(Vertex[] vertices, Vector3 center)
    {
        Vertices = vertices;
        Center = center;
    }

    public void Rotate(Vector3 point, Vector3 rotation)
    {
        foreach (var vertex in Vertices)
        {
            vertex.Rotate(point, rotation);
        }

        Center -= point;
        Center = VectorMath.RotateVector(Center, rotation);
        Center += point;
    }
    
    public void Move(Vector3 movement)
    {
        foreach (var vertex in Vertices)
        {
            vertex.Move(movement);
        }

        Center += movement;
    }

    public void Scale(float scalar)
    {
        Vector3 originalCenter = Center;
        Move(-Center);
        foreach (var vertex in Vertices)
        {
            vertex.Point1 *= scalar;
            vertex.Point2 *= scalar;
            vertex.Point3 *= scalar;
        }
        Move(originalCenter);
    }
}

static class VectorMath
{
    public static Vector3 RotateVector(Vector3 vector, Vector3 rotation)
    {

        Quaternion q = ToQuaternion(rotation.Z, rotation.Y, rotation.X);
        return Vector3.Transform(vector, q);
        
    }

    public static double[] PlaneEquation(Vertex vertex)
    {
        double[] final = new double[4];

        Vector3 crossOne = vertex.Point1 - vertex.Point2;
        Vector3 crossTwo = vertex.Point1 - vertex.Point3;

        Vector3 crossFinal = Vector3.Cross(crossOne, crossTwo);
        final[0] = crossFinal.X;
        final[1] = crossFinal.Y;
        final[2] = crossFinal.Z;
        final[3] = -(final[0] * vertex.Point1.X + final[1] * vertex.Point1.Y + final[2] * vertex.Point1.Z);
        return final;

    }

    public static Plane GetPlane(Vertex vertex)
    {
        Vector3 crossOne = vertex.Point1 - vertex.Point2;
        Vector3 crossTwo = vertex.Point1 - vertex.Point3;

        Vector3 crossFinal = Vector3.Cross(crossOne, crossTwo);
        float d = Vector3.Dot((crossOne - Vector3.Zero),  crossFinal) / crossFinal.Length();
        return new Plane(crossFinal, d);


    }

    public static Vector3 BarycentricCoords(Vector3 point, Vertex vertex)
    {
        Vector3 V0 = vertex.Point2 - vertex.Point1;
        Vector3 V1 = vertex.Point3 - vertex.Point1;
        Vector3 V2 = point - vertex.Point1;

        float dot00 = Vector3.Dot(V0, V0);
        float dot01 = Vector3.Dot(V0, V1);
        float dot02 = Vector3.Dot(V0, V2);
        float dot11 = Vector3.Dot(V1, V1);
        float dot12 = Vector3.Dot(V1, V2);

        float denom = dot00 * dot11 - dot01 * dot01;
        float alpha = (dot11 * dot02 - dot01 * dot12) / denom;
        float beta = (dot00 * dot12 - dot01 * dot02) / denom;
        float gamma = 1 - alpha - beta;
        return new Vector3(alpha,beta,gamma);

    }
    
    public static bool PointInVertex(Vector3 point, Vertex vertex)
    {
        
        Vector3 barycentric = BarycentricCoords(point, vertex);
        if (barycentric.X is > 0 and < 1 && barycentric.Y is > 0 and < 1 && barycentric.Z is > 0 and < 1)
        {
            return true;
        }

        return false;
    }
    
    static Quaternion ToQuaternion(float roll, float pitch, float yaw) // roll (x), pitch (Y), yaw (z)
    {
        // Abbreviations for the various angular functions

        float cr = MathF.Cos(roll * 0.5f);
        float sr = MathF.Sin(roll * 0.5f);
        float cp = MathF.Cos(pitch * 0.5f);
        float sp = MathF.Sin(pitch * 0.5f);
        float cy = MathF.Cos(yaw * 0.5f);
        float sy = MathF.Sin(yaw * 0.5f);
        
        float w = cr * cp * cy + sr * sp * sy;
        float x = sr * cp * cy - cr * sp * sy;
        float y = cr * sp * cy + sr * cp * sy;
        float z = cr * cp * sy - sr * sp * cy;
        
        return new Quaternion(x, y, z, w);
    }
    
}


//TODO: make more primitives and put them in the class instead of creating at runtime
static class Primitives
{
    public static Object CreateCube(List<Vertex> allVertices, float sideLength, Vector3 position, Vector3 rotation)
    {
        //back and front
        allVertices.Add(new Vertex( new Vector3(1,-1,-1), new Vector3(-1,1,-1),new Vector3(-1,-1,-1)));
        allVertices.Add(new Vertex(new Vector3(1,1,-1), new Vector3(-1,1,-1), new Vector3(1,-1,-1)));
        allVertices.Add(new Vertex( new Vector3(-1,-1,1), new Vector3(-1,1,1),new Vector3(1,-1,1)));
        allVertices.Add(new Vertex(new Vector3(1,-1,1), new Vector3(-1,1,1), new Vector3(1,1,1)));

        //top and bottom
        allVertices.Add(new Vertex(new Vector3(-1,1,-1), new Vector3(1,1,-1), new Vector3(1,1,1)));
        allVertices.Add(new Vertex(new Vector3(1,1,1), new Vector3(-1,1,1), new Vector3(-1,1,-1)));
        allVertices.Add(new Vertex(new Vector3(1,-1,1), new Vector3(1,-1,-1), new Vector3(-1,-1,-1)));
        allVertices.Add(new Vertex(new Vector3(-1,-1,-1), new Vector3(-1,-1,1), new Vector3(1,-1,1)));

        //left and right
        allVertices.Add(new Vertex(new Vector3(-1,1,1), new Vector3(-1,-1,1), new Vector3(-1,-1,-1)));
        allVertices.Add(new Vertex(new Vector3(-1,-1,-1), new Vector3(-1,1,-1), new Vector3(-1,1,1)));
        allVertices.Add(new Vertex(new Vector3(1,-1,-1), new Vector3(1,-1,1), new Vector3(1,1,1)));
        allVertices.Add(new Vertex(new Vector3(1,1,1), new Vector3(1,1,-1),new Vector3(1,-1,-1)));

        Object cube = new Object(allVertices.GetRange(allVertices.Count - 12, 12).ToArray(), new Vector3(0,0,0));
        cube.Rotate(new Vector3(0,0,0), rotation);
        cube.Move(position);
        cube.Scale(0.5f * sideLength);
        
        return cube;
    }
}

class Camera
{
    public Ray CameraCenter;

    public int Fov;
    public int CameraWidth;
    public float ViewDistance;
    public float DegreesPerPixel;
    public List<Ray> AllCameraRays;

    public Camera(Ray cameraCenter, int fov, int cameraWidth, float viewDistance)
    {
        Fov = fov;
        CameraCenter = cameraCenter;
        ViewDistance = viewDistance;
        CameraWidth = cameraWidth;
        
        ResetCamera();

        
    }
    private void ResetCamera()
    {
        //here we split up the square of the screen into pixels and give each pixel a ray to find its value
        float degreesPerPixel = (float)Fov / CameraWidth;
        AllCameraRays = new List<Ray>();
        for (int i = -CameraWidth/2; i < CameraWidth/2; i++)
        {
            for (int j = -CameraWidth/2; j < CameraWidth/2; j++)
            {
                Vector3 rotation = new Vector3(0, -i * degreesPerPixel * ((2 * (float)Math.PI)/360), -j * degreesPerPixel * ((2 * (float)Math.PI)/360));
                AllCameraRays.Add(new Ray(CameraCenter.Origin, VectorMath.RotateVector(CameraCenter.Direction, rotation))); 
            }
    
        }
    }


}


