using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

namespace Tallow
{
    public static class Useful
    {
        public static float Percent(this float f, float percent)
        {
            return (f * percent) * 0.01f;
        }
        public static double Percent(this double f, double percent)
        {
            return (f * percent) * 0.01f;
        }

        public static Vector3 DirTo<T, O>(this T firstThing, O otherThing) where T : Component where O : Component
        {
            // Calculate the direction to the player
            return (otherThing.transform.position - firstThing.transform.position).normalized;
        }

        public static Vector3 DirTo(this Vector3 firstThing, Vector3 otherThing)
        {
            // Calculate the direction to the player
            return (otherThing - firstThing).normalized;
        }

        public struct CenterOfVectorsJob : IJob
        {
            [ReadOnly] public NativeArray<Vector3> points;
            public Vector3 center;

            public void Execute()
            {
                // If there's only one point, set it as the center
                if (points.Length == 1)
                {
                    center = points[0];
                    return;
                }

                // Sum all points
                for (int i = 0; i < points.Length; i++)
                {
                    center += points[i];
                }

                // Divide by the number of points to get the average
                center /= points.Length;
            }
        }
        public static Vector3 CalculateCenter(this List<Vector3> points)
        {
            NativeArray<Vector3> nativePoints = new NativeArray<Vector3>(points.ToArray(), Allocator.TempJob);
            Vector3 centerReport = Vector3.zero;

            CenterOfVectorsJob centerJob = new CenterOfVectorsJob
            {
                points = nativePoints,
                center = centerReport
            };

            JobHandle handle = centerJob.Schedule();
            handle.Complete();

            nativePoints.Dispose();

            return centerReport;
        }

        public static bool InRange(this float f, float min, float max)
        {
            if (f < min || f > max) return false;
            return true;
        }

        //Checks if the component has anything between it and something else.
        public static bool CanSee<T, O>(this T thisThing, O otherThing, float range) where T : Component where O : Component
        {
            Vector3 tfPos = thisThing.transform.position;

            //If we hit something that wasn't what we were aiming for, then return true to the view being blocked.
            if (Physics.Raycast(tfPos, tfPos.DirTo(otherThing.transform.position), out RaycastHit hit, range)
                && hit.collider.gameObject != otherThing)
            {
                return false;
            }

            return true;
        }

        //Checks if the component has anything between it and something else.
        public static bool CanSee<T, O>(this T thisThing, O otherThing) where T : Component where O : Component
        {
            Vector3 tfPos = thisThing.transform.position;

            //If we hit something that wasn't what we were aiming for, then return true to the view being blocked.
            if (Physics.Raycast(tfPos, tfPos.DirTo(otherThing.transform.position), out RaycastHit hit)
                && hit.collider.gameObject != otherThing)
            {
                return false;
            }

            return true;
        }

        public static bool CanSeeFullscan<T, O>(this T thisThing, O otherThing) where T : Component where O : Component
        {
            // Calculate direction to the target
            Vector3 direction = thisThing.DirTo(otherThing);

            // Perform raycast to check for obstacles
            RaycastHit hit;

            if (Physics.Raycast(thisThing.transform.position, direction, out hit, Mathf.Infinity))
            {
                if (hit.transform.root.gameObject == otherThing.gameObject) return true; 

                // Check over each of the gameObjects within the 
                foreach (Transform t in hit.transform.root)
                {
                    // If an obstacle is hit, the target is not visible
                    if (t.gameObject == otherThing.gameObject)
                    {
                        return true;
                    }
                }
            }

            return false; // View is blocked or no hits
        }

        public static bool CanSeeFullscan<T, O>(this T thisThing, O otherThing, LayerMask obstacleMask) where T : Component where O : Component
        {
            {
                // Calculate direction to the target
                Vector3 direction = thisThing.transform.DirTo(otherThing);

                // Perform raycast to check for obstacles
                RaycastHit hit;

                if (Physics.Raycast(thisThing.transform.position, direction, out hit, Mathf.Infinity, obstacleMask))
                {
                    // Check over each of the gameObjects within the 
                    foreach (Transform t in hit.transform.root)
                    {
                        // If an obstacle is hit, the target is not visible
                        if (t == thisThing.transform)
                        {
                            return true;
                        }
                    }
                }
            }

            return false; // View is blocked or no hits
        }

        public static RaycastHit[] AllHits<T, O>(this T thisThing, O otherThing, int maxTargets) where T : Component where O : Component
        {
            Vector3 thisPosition = thisThing.transform.position;
            Vector3 direction = thisPosition.DirTo(otherThing.transform.position);

            RaycastHit[] hits = new RaycastHit[maxTargets];

            // Perform raycast from thisThing towards otherThing
            Physics.RaycastNonAlloc(thisPosition, direction, hits, float.MaxValue);

            return hits;
        }

        public struct DistanceJob : IJob
        {
            public Vector3 positionA;
            public Vector3 positionB;
            public NativeArray<float> result;

            public void Execute()
            {
                result[0] = Vector3.Distance(positionA, positionB);
            }
        }
        public static float DistAsync(Vector3 positionA, Vector3 positionB)
        {
            // Create NativeArray to store result
            NativeArray<float> result = new NativeArray<float>(1, Allocator.TempJob);

            DistanceJob distanceJob = new DistanceJob
            {
                positionA = positionA,
                positionB = positionB,
                result = result
            };

            JobHandle handle = distanceJob.Schedule();
            handle.Complete();

            float distance = result[0];

            result.Dispose();

            return distance;
        }

        public static float DistTo<T, O>(this T to, O distToThis) where T : Component where O : Component
        {
            return DistAsync(to.transform.position, distToThis.transform.position);
        }

        public static float DistTo<T>(this T to, Vector3 distToThis) where T : Component
        {
            return DistAsync(to.transform.position, distToThis);
        }

        public static float DistTo(this Vector3 to, Vector3 distToThis)
        {
            return DistAsync(to, distToThis);
        }

        public static void SafeAdd<T>(this List<T> toThis, T addThis)
        {
            if (!toThis.HasThis(addThis)) toThis.Add(addThis);
        }

        public static void SafeRemove<T>(this List<T> fromThis, T removeThis)
        {
            if (fromThis.HasThis(removeThis)) fromThis.Remove(removeThis);
        }

        public static Dictionary<TKey, TValue> Sort<TKey, TValue>(this Dictionary<TKey, TValue> dictionary)
        {
            return dictionary.OrderBy(x => x.Value).ToDictionary(pair => pair.Key, pair => pair.Value);
        }

        public static Dictionary<TKey, TValue> SortByLargest<TKey, TValue>(this Dictionary<TKey, TValue> dictionary)
        {
            return dictionary.OrderByDescending(x => x.Value).ToDictionary(pair => pair.Key, pair => pair.Value);
        }

        public static List<T> GetThingsInCone<T>(this Transform thisThing, List<T> these, float coneAngle) where T : Component
        {
            List<T> those = new List<T>();

            foreach (T t in these)
            {
                if (thisThing.InCone(t, coneAngle)) those.Add(t);
            }
            return those;
        }

        public static bool InCone<T, O>(this T thisThing, O otherThing, float coneAngle) where T : Component where O : Component
        {
            float angle = thisThing.transform.forward.AngleTo(thisThing.DirTo(otherThing));

            if (angle < coneAngle) return true;
            return false;
        }

        public static bool Empty<T>(this List<T> list)
        {
            if (list.Count == 0) return true;
            return false;
        }

        public static bool TryGetComponentFullscan<T>(this Component go, out T comp) where T : Component
        {
            T t = go.GetComponent<T>();

            if (t)
            {
                comp = t;
                return true;
            }
            else
            {
                t = go.transform.root.GetComponentInChildren<T>();

                if (t)
                {
                    comp = t;
                    return true;
                }
            }

            comp = t;
            return false;
        }

        public static bool TryGetComponentInParents<T>(this Component go, out T comp) where T : Component
        {
            List<Transform> parents = GetParents();

            List<Transform> GetParents()
            {
                List<Transform> LT = new List<Transform>();

                foreach (Transform t in go.transform.root.transform)
                {
                    //Stop trying to find things once we reach the child component of origin.
                    if (go.gameObject == t.gameObject)
                    {
                        return LT;
                    }
                    else LT.Add(t);
                }

                return LT;
            }

            int parentCount = parents.Count();

            //Strange alteration to the typical for loop.
            //Start with the very last object in the list of parents, which would be the first parent above the original component.
            //Then, work up until you reach the last parent.
            for (int i = parentCount; i > parentCount; i--)
            {
                Transform t = parents[i];

                if (t.TryGetComponent<T>(out T tea))
                {
                    comp = tea;
                    return true;
                }
            }

            comp = null;
            return false;
        }

        public static bool TryGetComponentsInParents<T>(this Component go, out List<T> comp) where T : Component
        {
            List<Transform> parents = GetParents();

            List<Transform> GetParents()
            {
                List<Transform> LT = new List<Transform>();

                foreach (Transform t in go.transform.root)
                {
                    //Stop trying to find things once we reach the child component of origin.
                    if (go.gameObject == t.gameObject)
                    {
                        return LT;
                    }
                    else LT.Add(t);
                }

                return LT;
            }

            int parentCount = parents.Count();
            List<T> retList = new List<T>();

            //Strange alteration to the typical for loop.
            //Start with the very last object in the list of parents, which would be the first parent above the original component.
            //Then, work up until you reach the last parent.
            for (int i = parentCount; i > parentCount; i--)
            {
                Transform t = parents[i];

                if (t.TryGetComponent<T>(out T tea))
                {
                    retList.Add(tea);
                }
            }

            if (retList.Count > 0)
            {
                comp = retList;
                return true;
            }

            comp = null;
            return false;
        }

        public static bool TryGetComponentInChildren<T>(this Component go, out T comp) where T : Component
        {
            List<T> LT = new List<T>(ChildrenList());

            List<T> ChildrenList()
            {
                List<T> CofC = new List<T>();

                RigB(go.transform);

                void RigB(Transform parent)
                {
                    //For each child in this gameobject heirarchy.
                    foreach (Transform t in parent)
                    {
                        if (t.TryGetComponent<T>(out T tr))
                        {
                            CofC.Add(tr);
                        }

                        RigB(t.transform);
                    }
                }

                return CofC;
            }

            if (LT.Count > 0)
            {
                comp = LT[0];
                return true;
            }

            comp = null;
            return false;
        }

        public static bool TryGetComponentsInChildren<T>(this Component go, out List<T> comps) where T : Component
        {
            List<T> LT = new List<T>(ChildrenList());

            List<T> ChildrenList()
            {
                List<T> CofC = new List<T>();

                RigB(go.transform);

                void RigB(Transform parent)
                {
                    //For each child in this gameobject heirarchy.
                    foreach (Transform t in parent)
                    {
                        if (t.TryGetComponent<T>(out T tr))
                        {
                            CofC.Add(tr);
                        }

                        RigB(t.transform);
                    }
                }

                return CofC;
            }

            if (LT.Count > 0)
            {
                comps = LT;
                return true;
            }

            comps = null;
            return false;
        }

        public static int RandRange(int first, int second)
        {
            return UnityEngine.Random.Range(first, second);
        }
        public static float RandRange(float first, float second)
        {
            return UnityEngine.Random.Range(first, second);
        }
        public static double RandRange(double first, double second)
        {
            return (double)UnityEngine.Random.Range((float)first, (float)second);
        }

        public static IEnumerator DestroyAfter<T>(this T wipeThis, float afterThisTime) where T : Component
        {
            //Wait until the timer has depleted entirely before continuing.
            while (afterThisTime > 0)
            {
                afterThisTime -= Time.deltaTime;
                yield return null;
            }

            if (wipeThis) MonoBehaviour.Destroy(wipeThis.gameObject);
        }

        public static IEnumerator DestroyAfter(this GameObject wipeThis, float afterThisTime)
        {
            //Wait until the timer has depleted entirely before continuing.
            while (afterThisTime > 0)
            {
                afterThisTime -= Time.deltaTime;
                yield return null;
            }

            if (wipeThis) MonoBehaviour.Destroy(wipeThis);
        }

        public static IEnumerator DestroyAtDistance<T>(this T wipeThis, float afterThisDist, Vector3 fromThis) where T : Component
        {
            float dist = wipeThis.DistTo(fromThis);

            //Wait until the timer has depleted entirely before continuing.
            //Also, check to make sure the fromThis transform still exists and hasn't been destroyed. If it has, exit this routine and log an error.
            while (dist < afterThisDist)
            {
                dist = wipeThis.DistTo(fromThis);
                yield return null;
            }

            if (wipeThis) MonoBehaviour.Destroy(wipeThis.gameObject);
        }

        //Makes sure that the gameObject is the weight specified, equally distributed across all rigidbodies.
        public static void SetEvenSkeletonWeight(this MonoBehaviour mb, float fullWeight)
        {
            //Initializes the coroutine on the monobehavior that called it to be activated.
            mb.StartCoroutine(D(mb.transform.parent));

            IEnumerator D(Transform parent)
            {
                List<Rigidbody> skeletonRBList = new List<Rigidbody>();
                RigB(parent);

                //Set the mass of the entire body equally set between all rigidBodies in the heirarchy.
                foreach (Rigidbody rb in skeletonRBList)
                {
                    rb.mass = fullWeight / skeletonRBList.Count;
                }

                void RigB(Transform parent)
                {
                    //For each child in this gameobject heirarchy.
                    foreach (Transform t in parent)
                    {
                        if (t.TryGetComponent<Rigidbody>(out Rigidbody RB))
                        {
                            skeletonRBList.Add(RB);
                        }

                        RigB(t);
                    }
                }

                yield return null;
            }
        }

        public static float Speed(this Rigidbody RB) { return RB.velocity.magnitude; }

        public static bool TestFate(float f)
        {
            if (f > RandRange(0, 100))
            {
                return true;
            }
            return false;
        }
        public struct ClosestObjectJob<T> : IJobParallelFor where T : Component
        {
            [ReadOnly] public NativeArray<Vector3> positions;
            public Vector3 closestTo;
            public NativeArray<float> distances;
            public NativeArray<int> closestIndices;

            public void Execute(int index)
            {
                distances[index] = positions[index].DistTo(closestTo);
            }
        }
        public static T GetClosest<T>(this Vector3 closestTo, List<T> components) where T : Component
        {
            int count = components.Count;
            if (count == 0) return null;

            NativeArray<Vector3> positions = new NativeArray<Vector3>(count, Allocator.TempJob);
            NativeArray<float> distances = new NativeArray<float>(count, Allocator.TempJob);
            NativeArray<int> closestIndices = new NativeArray<int>(1, Allocator.TempJob);

            // Copy positions to NativeArray
            for (int i = 0; i < count; i++)
            {
                positions[i] = components[i].transform.position;
            }

            // Schedule job to compute distances in parallel
            ClosestObjectJob<T> job = new ClosestObjectJob<T>
            {
                positions = positions,
                closestTo = closestTo,
                distances = distances,
                closestIndices = closestIndices
            };
            JobHandle handle = job.Schedule(count, 64);

            // Wait for job to complete
            handle.Complete();

            // Find the closest object
            float minDist = float.MaxValue;
            int closestIndex = -1;
            for (int i = 0; i < count; i++)
            {
                float dist = distances[i];
                if (dist < minDist)
                {
                    minDist = dist;
                    closestIndex = i;
                }
            }

            // Cleanup
            positions.Dispose();
            distances.Dispose();
            closestIndices.Dispose();

            if (closestIndex >= 0)
            {
                return components[closestIndex];
            }
            else
            {
                return null;
            }
        }

        public struct AngleJob : IJob
        {
            public Vector3 from;
            public Vector3 to;
            public NativeArray<float> result;

            public void Execute()
            {
                float num = Mathf.Sqrt(from.sqrMagnitude * to.sqrMagnitude);
                if (num < 1E-15f)
                {
                    result[0] = 0f;
                    return;
                }

                float num2 = Mathf.Clamp(Vector3.Dot(from, to) / num, -1f, 1f);
                result[0] = Mathf.Acos(num2) * Mathf.Rad2Deg;
            }
        }

        public static float AngleToAsync(this Vector3 from, Vector3 to)
        {
            NativeArray<float> result = new NativeArray<float>(1, Allocator.TempJob);

            AngleJob angleJob = new AngleJob
            {
                from = from,
                to = to,
                result = result
            };

            JobHandle handle = angleJob.Schedule();
            handle.Complete();

            float angle = result[0];

            result.Dispose();

            return Mathf.Abs(angle);
        }
        
        public static float AngleTo(this Vector3 thing, Vector3 otherThing)
        {
            return Mathf.Abs(Vector3.Angle(thing, otherThing));
        }

        // Function to calculate angle between two 2D vectors
        public static float AngleTo(Vector2 a, Vector2 b)
        {
            // Calculate dot product
            float dotProduct = Vector2.Dot(a.normalized, b.normalized);

            // Calculate angle in radians, then convert to Degrees.
            float angle = Mathf.Acos(dotProduct) * Mathf.Rad2Deg;

            return Mathf.Abs(angle);
        }

        /// <summary>
        /// Returns the closest component, by distance, to the component specified fron the list specified.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <typeparam name="O"></typeparam>
        /// <param name="LGO"></param>
        /// <param name="closestTo"></param>
        /// <returns></returns>

        public static void TurnTowards<T, O>(this T thingToRotate, O turnTowards, float rotationSpeed) where T : Component where O : Component
        {
            Quaternion targetRotation = Quaternion.LookRotation(thingToRotate.transform.position.DirTo(turnTowards.transform.position));

            // Slerp to smoothly interpolate between the current rotation and the target rotation
            thingToRotate.transform.rotation = Quaternion.Slerp(thingToRotate.transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
        }

        public static void TurnTowards<T>(this T thingToRotate, Vector3 turnTowards, float rotationSpeed) where T : Component
        {
            Quaternion targetRotation = Quaternion.LookRotation(thingToRotate.transform.position.DirTo(turnTowards));

            // Slerp to smoothly interpolate between the current rotation and the target rotation
            thingToRotate.transform.rotation = Quaternion.Slerp(thingToRotate.transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
        }

        public static void ResetRotation<T>(this T thingToRotate, float rotationSpeed) where T : Component
        {
            // Slerp to smoothly interpolate between the current rotation and the target rotation
            thingToRotate.transform.localRotation = Quaternion.Slerp(thingToRotate.transform.localRotation, new Quaternion(0, 0, 0, 0), rotationSpeed * Time.deltaTime);
        }

        public static void TurnTowardsClamped<T, O>(this T thingToRotate, O turnTowards, Vector3 min, Vector3 max, float rotationSpeed)
            where T : Component
            where O : Component
        {
            Vector3 dir = turnTowards.transform.position.DirTo(thingToRotate.transform.position);

            float x = Mathf.Clamp(dir.x, min.x, max.x);
            float y = Mathf.Clamp(dir.y, min.y, max.y);
            float z = Mathf.Clamp(dir.z, min.z, max.z);

            Quaternion targetRotation = Quaternion.LookRotation(new Vector3(x, y, z));

            // Slerp to smoothly interpolate between the current rotation and the target rotation
            thingToRotate.transform.rotation = Quaternion.Slerp(thingToRotate.transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
        }

        public static void TurnTowardsClamped<T, O>(this T thingToRotate, O turnTowards, Vector3 minMax, float rotationSpeed)
            where T : Component
            where O : Component
        {
            Vector3 dir = turnTowards.transform.position.DirTo(thingToRotate.transform.position);

            float x = Mathf.Clamp(dir.x, -minMax.x, minMax.x);
            float y = Mathf.Clamp(dir.y, -minMax.y, minMax.y);
            float z = Mathf.Clamp(dir.z, -minMax.z, minMax.z);

            Quaternion targetRotation = Quaternion.LookRotation(new Vector3(x, y, z));

            // Slerp to smoothly interpolate between the current rotation and the target rotation
            thingToRotate.transform.rotation = Quaternion.Slerp(thingToRotate.transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
        }

        public static void TurnTowardsClamped<T, O>(this T thingToRotate, O turnTowards, float minMax, float rotationSpeed)
            where T : Component
            where O : Component
        {
            Vector3 dir = turnTowards.transform.position.DirTo(thingToRotate.transform.position);

            float x = Mathf.Clamp(dir.x, -minMax, minMax);
            float y = Mathf.Clamp(dir.y, -minMax, minMax);
            float z = Mathf.Clamp(dir.z, -minMax, minMax);

            Quaternion targetRotation = Quaternion.LookRotation(new Vector3(x, y, z));

            // Slerp to smoothly interpolate between the current rotation and the target rotation
            thingToRotate.transform.rotation = Quaternion.Slerp(thingToRotate.transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
        }

        public static void TurnTowardsDirection<T, O>(this T thingToRotate, Vector3 direction, float rotationSpeed) where T : Component where O : Component
        {
            Quaternion targetRotation = Quaternion.LookRotation(direction);

            // Slerp to smoothly interpolate between the current rotation and the target rotation
            thingToRotate.transform.rotation = Quaternion.Slerp(thingToRotate.transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
        }

        public static void PushTowards<T>(this Rigidbody thingToPush, T pushTowards, float floatForce) where T : Component
        {
            Vector3 dir = thingToPush.transform.position.DirTo(pushTowards.transform.position);

            // Calculate the force to be applied (constant acceleration)
            Vector3 accelerationForce = dir * floatForce;

            // Apply the acceleration force to the Rigidbody
            thingToPush.AddForce(accelerationForce, ForceMode.Force);
        }

        public static void PushTowards(this Rigidbody thingToPush, Vector3 pushTowards, float floatForce)
        {
            Vector3 dir = thingToPush.transform.position.DirTo(pushTowards);

            // Calculate the force to be applied (constant acceleration)
            Vector3 accelerationForce = dir * floatForce;

            // Apply the acceleration force to the Rigidbody
            thingToPush.AddForce(accelerationForce, ForceMode.Force);
        }

        public static void PushTowardsDirection(this Rigidbody thingToPush, Vector3 direction, float floatForce)
        {
            // Calculate the force to be applied (constant acceleration)
            Vector3 accelerationForce = direction * floatForce;

            // Apply the acceleration force to the Rigidbody
            thingToPush.AddForce(accelerationForce, ForceMode.Force);
        }

        //UNTESTED
        // 
        //Get the thickness of a collider through a line.
        public static void CheckColliderThickness(this Collider colliderToCheck, Transform rayOrigin, Vector3 rayDirection)
        {
            RaycastHit hit;

            if (colliderToCheck.Raycast(new Ray(rayOrigin.position, rayDirection), out hit, Mathf.Infinity))
            {
                // Ray hit the collider, get hit point
                Vector3 entryPoint = hit.point;

                // Cast another ray in the opposite direction
                if (colliderToCheck.Raycast(new Ray(entryPoint - rayDirection * 10f, -rayDirection), out hit, Mathf.Infinity))
                {
                    // Ray hit the collider again, get hit point
                    Vector3 exitPoint = hit.point;

                    // Calculate thickness
                    float thickness = Vector3.Distance(entryPoint, exitPoint);

                    Debug.Log("Collider thickness: " + thickness);
                }
                else
                {
                    Debug.LogError("Ray didn't hit the collider from the inside!");
                }
            }
            else
            {
                Debug.LogError("Ray didn't hit the collider!");
            }
        }
    }

    public static class ListWorks
    {
        public static bool HasThis<T>(this List<T> list, T thing)
        {
            if (list.IndexOf(thing) != -1) return true;
            return false;
        }

        // Define a custom comparer for components
        public class ComponentComparer<T> : IComparer<T> where T : Component
        {
            public int Compare(T x, T y)
            {
                // Implement comparison logic here
                // For example, compare by name
                return x.name.CompareTo(y.name);
            }
        }

        public static List<T> ListSort<T>(this List<T> list) where T : Component
        {
            // Provide a custom comparison function to the OrderBy method
            return list.OrderBy(x => x, new ComponentComparer<T>()).ToList();
        }

        //Sorts a list by string, then returns the entire sorted list.
        public static List<T> SortByName<T>(List<T> list, string name)
        {
            List<T> sortedList = new List<T>(list);

            // Check if the type T has a property named "name"
            var nameProperty = typeof(T).GetProperty("name");

            if (nameProperty != null)
            {
                // Filter out items whose names do not contain the provided string S
                sortedList = sortedList.Where(item => nameProperty.GetValue(item).ToString().Contains(name)).ToList();
            }

            // Sort the list alphabetically by name
            sortedList.Sort((a, b) => nameProperty.GetValue(a).ToString().CompareTo(nameProperty.GetValue(b).ToString()));

            return sortedList;
        }
        public static List<T> SortByDistance<T>(T closestTo, List<T> list, Func<T, Vector3> getPosition)
        {
            Dictionary<T, float> distances = new Dictionary<T, float>();

            // Calculate distances from 'closestTo' for each item in the list
            foreach (T item in list)
            {
                distances.Add(item, getPosition(closestTo).DistTo(getPosition(item)));
            }

            // Sort the dictionary by distance and return the sorted keys
            return distances.OrderBy(x => x.Value).Select(x => x.Key).ToList();
        }

        public static List<T> SortByDistance<T>(this Vector3 closestTo, List<T> LGO) where T : Component
        {
            Dictionary<T, float> DistOfGameObjs = new Dictionary<T, float>();

            foreach (T GO in LGO)
            {
                DistOfGameObjs.Add(GO, closestTo.DistTo(GO.transform.position));
            }

            // Sort the dictionary by value of distance, then return the first of these values.
            return DistOfGameObjs.OrderBy(x => x.Value).ToDictionary(pair => pair.Key, pair => pair.Value).Keys.ToList();
        }

        public static List<Vector3> SortByDistance(this Vector3 closestTo, List<Vector3> LGO)
        {
            Dictionary<Vector3, float> DistOfGameObjs = new Dictionary<Vector3, float>();

            foreach (Vector3 GO in LGO)
            {
                DistOfGameObjs.Add(GO, closestTo.DistTo(GO));
            }

            // Sort the dictionary by value of distance, then return the first of these values.
            return DistOfGameObjs.OrderBy(x => x.Value).ToDictionary(pair => pair.Key, pair => pair.Value).Keys.ToList();
        }
        public struct DistanceCalculationJob<T> : IJobParallelFor where T : Component
        {
            [ReadOnly] public NativeArray<Vector3> positions;
            public Vector3 point;
            public NativeArray<float> distances;

            public void Execute(int index)
            {
                distances[index] = Vector3.Distance(positions[index], point);
            }
        }
        public static List<T> SortByDistance<T>(this List<T> components, Vector3 point) where T : Component
        {
            int count = components.Count;
            if (count == 0) return components;

            NativeArray<Vector3> positions = new NativeArray<Vector3>(count, Allocator.TempJob);
            NativeArray<float> distances = new NativeArray<float>(count, Allocator.TempJob);

            // Copy positions to NativeArray
            for (int i = 0; i < count; i++)
            {
                positions[i] = components[i].transform.position;
            }

            // Schedule job to compute distances in parallel
            DistanceCalculationJob<T> job = new DistanceCalculationJob<T>
            {
                positions = positions,
                point = point,
                distances = distances
            };
            JobHandle handle = job.Schedule(count, 64);

            // Wait for job to complete
            handle.Complete();

            // Create a copy of the original list
            List<T> sortedComponents = new List<T>(components);

            sortedComponents.ToArray().SortAsync(distances.ToArray());

            // Cleanup
            positions.Dispose();
            distances.Dispose();

            return sortedComponents;
        }
        public static List<T> SiftForType<T>(List<UnityEngine.Object> objectList) where T : UnityEngine.Object
        {
            // Use LINQ to filter out objects of type T and convert them to List<T>
            List<T> filteredList = objectList
                .Where(obj => obj != null && obj.GetType() == typeof(T))
                .Select(obj => obj as T)
                .ToList();

            return filteredList;
        }
        public static List<T> TryGetNumberOfList<T>(this List<T> list, int quant)
        {
            if (quant >= list.Count) return list;

            List<T> rL = new List<T>();

            if (list.Count == 0) return rL;

            for (int i = 0; i < quant; i++)
            {
                // Ensure not to exceed the bounds of the input list
                if (i < list.Count)
                {
                    rL.Add(list[i]);
                }
                else break; // Exit the loop if we reached the end of the input list
            }

            return rL;
        }

        public static T GetRandom<T>(this List<T> list)
        {
            return list[Useful.RandRange(0, list.Count)];
        }

        public static List<T> TryGetPercentOfList<T>(this List<T> rbList, float percent)
        {
            List<T> newList = new List<T>();

            //Check to see if it needs to do any of the calculations below.
            if (Mathf.Approximately(percent, 100)) return rbList;
            if (Mathf.Approximately(percent, 0)) return newList;

            List<T> rbLeftToIterate = new List<T>(rbList);

            int numberOfProps = Mathf.RoundToInt(percent * rbList.Count);

            //Interate through the entire list of objects, selecting randomly from all props that have not been selected already.
            for (int i = 0; i < numberOfProps; i++)
            {
                int propInt = UnityEngine.Random.Range(0, rbLeftToIterate.Count);
                T RBody = rbLeftToIterate[propInt];

                newList.Add(RBody);
                rbLeftToIterate.RemoveAt(propInt);
            }

            return newList;
        }

        //Compares two lists of any type against their length and individual components.
        public static bool Compare<T1, T2>(this List<T1> firstList, List<T2> nextList)
            where T1 : IEquatable<T2>
        {
            if (firstList.Count != nextList.Count)
                return false; // If they're not the same length, return false.

            for (int i = 0; i < firstList.Count; i++)
            {
                if (!firstList[i].Equals(nextList[i]))
                    return false; // If any elements are not equal, return false.
            }

            return true; // If all elements are equal, return true.
        }

        //Returns a list of words in a string.
        //Works by separating the words from their respective spaces before returning the series of characters between this space the previous space.
        public static List<string> ExtractWords(this string sentence)
        {
            List<string> newWords = new List<string>();
            List<char> newWord = new List<char>();

            // Iterate through the characters of the input sentence
            foreach (char c in sentence)
            {
                // If the character isn't a space, add it to the current word
                if (c != ' ')
                {
                    newWord.Add(c);
                }
                else if (newWord.Count > 0)
                {
                    // If we encounter a space and there are characters in newWord,
                    // it means we've reached the end of a word
                    string word = new string(newWord.ToArray()); // Convert the list of characters to a string
                    newWords.Add(word); // Add the word to the list of words
                    newWord.Clear(); // Clear the characters in newWord to start a new word
                }
            }

            // Add the last word if it exists
            if (newWord.Count > 0)
            {
                string word = new string(newWord.ToArray());
                newWords.Add(word);
            }

            return newWords;
        }

        //If trying to get something from a list, this script will interate through said list and return whatever type you wanted from the children.
        public static List<T> TryGetComponentsFromChildren<T>(this GameObject GO) where T : Component
        {
            List<T> LRB = new List<T>();

            foreach (GameObject TO in GO.transform)
            {
                if (TO.TryGetComponent<T>(out T c))
                {
                    LRB.Add(c);
                }
            }
            return LRB;
        }

        public static string GetNamesFromList<T>(this List<T> list)
        {
            string ret = "";

            for (int i = 0; i < list.Count; i++)
            {
                // Convert the item to a string representation and append it to ret
                ret += list[i].ToString();

                // Add a space if it's not the last item in the list
                if (i < list.Count - 1)
                {
                    ret += ", ";
                }
            }

            return ret;
        }
    }

    public static class WaveWorks
    {
        public static float GetBrightness(float intensity, float distance)
        {
            // Calculate brightness using the inverse square law
            float brightness = intensity / (distance * distance);

            return Mathf.Abs(brightness);
        }
        public static double GetBrightness(double intensity, double distance)
        {
            // Calculate brightness using the inverse square law
            double brightness = intensity / (distance * distance);

            return Math.Abs(brightness);
        }

        public static float GetBrightness(this Light light, float distance)
        {
            // Calculate brightness using the inverse square law
            float brightness = light.intensity / (distance * distance);

            return Mathf.Abs(brightness);
        }

        /// <summary>
        /// Calculates the brightness of an object relative to the observer
        /// </summary>
        /// <param name="light"></param>
        /// <param name="distance"></param>
        /// <returns></returns>
        public static float GetBrightnessRelative(this Transform observer, Transform thingShinedOn, Transform thingShining, float intensity)
        {
            // Get distance from the radiation source to the object it is shining on.
            float distToObj = Mathf.Abs(thingShinedOn.DistTo(thingShining));

            // Get distance from the observer to the object it is shining on.
            float distToObs = Mathf.Abs(observer.DistTo(thingShinedOn));

            // Calculate brightness using the inverse square law
            return intensity / (distToObj * distToObj) * (distToObs * distToObs);
        }
        /// <summary>
        /// Calculates at what distance the light reflected off of an object will reach a certain intensity.
        /// </summary>
        /// <param name="initialIntensity"> Initial intensity of the light source. </param>
        /// <param name="reflectivity"> Overall reflectivity of the object, between 0 and 1. </param>
        /// <param name="desiredIntensity"> Intensity we want to achieve. </param>
        /// <returns></returns>
        public static float IntensityAtDistance(float initialIntensity, float reflectivity, float desiredIntensity)
        {
            // Calculate the distance
            return Mathf.Sqrt((initialIntensity * reflectivity) / desiredIntensity);
        }

        public static List<RadReturn> SortByDoppler(this List<RadReturn> rList, Rigidbody thisRB, float dMin, float dMax)
        {
            int count = rList.Count;

            // Initialize result array with enough capacity
            NativeArray<float> result = new NativeArray<float>(count * 64, Allocator.TempJob);

            Vector3 thisVel = thisRB.velocity;
            Vector3 thisPos = thisRB.transform.position;

            for (int i = 0; i < count; i++)
            {
                Vector3 otherVel = rList[i].rigidBody.velocity;
                Vector3 otherPos = rList[i].transform.position;

                DopplerJob dopplerJob = new DopplerJob
                {
                    thisRBVel = thisVel,
                    otherRBVel = otherVel,
                    thisPos = thisPos,
                    otherPos = otherPos,
                    minDop = dMin,
                    maxDop = dMax,

                    // Pass the correct offset into the result array for each job
                    result = result.GetSubArray(i * 64, 64)
                };

                JobHandle handle = dopplerJob.Schedule(); // Schedule one job at a time
                handle.Complete();
            }

            // Create a copy of the original list
            List<RadReturn> sortedComponents = new List<RadReturn>(rList);

            sortedComponents.ToArray().SortDescendingAsync(result.ToArray());

            result.Dispose();

            return sortedComponents;
        }
        /// <summary>
        /// Sorts one list based on values provided by another.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="sortedComponents"></param>
        /// <param name="result"></param>
        public static async void SortAsync<T>(this T[] sortedComponents, float[] result) where T : Component
        {
            await Task.Run(() =>
            {
                int count = Mathf.Min(result.Length, sortedComponents.Length); // Ensure count doesn't exceed array lengths
                                                                               // Sort the list based on bearings
                for (int i = 0; i < count - 1; i++)
                {
                    for (int j = i + 1; j < count; j++)
                    {
                        if (j < result.Length && j < sortedComponents.Length) // Check bounds
                        {
                            if (result[i] < result[j])
                            {
                                // Swap elements using tuple
                                (sortedComponents[i], sortedComponents[j]) = (sortedComponents[j], sortedComponents[i]);
                                (result[i], result[j]) = (result[j], result[i]);
                            }
                        }
                    }
                }
            });
        }

        public static async void SortDescendingAsync<T>(this T[] sortedComponents, float[] result) where T : Component
        {
            await Task.Run(() =>
            {
                int count = Mathf.Min(result.Length, sortedComponents.Length); // Ensure count doesn't exceed array lengths
                                                                               // Sort the list based on bearings
                for (int i = 0; i < count - 1; i++)
                {
                    for (int j = i + 1; j < count; j++)
                    {
                        if (j < result.Length && j < sortedComponents.Length) // Check bounds
                        {
                            if (result[i] > result[j])
                            {
                                // Swap elements using tuple
                                (sortedComponents[i], sortedComponents[j]) = (sortedComponents[j], sortedComponents[i]);
                                (result[i], result[j]) = (result[j], result[i]);
                            }
                        }
                    }
                }
            });
        }
        public struct DopplerJob : IJob
        {
            public Vector3 thisRBVel;
            public Vector3 thisPos;
            public Vector3 otherRBVel;
            public Vector3 otherPos;
            public NativeArray<float> result;
            public float minDop;
            public float maxDop;

            public void Execute()
            {
                // Calculate Doppler shift and check if it's within the specified range
                float dopplerValue = Vector3.Dot(otherRBVel - thisRBVel, thisPos.DirTo(otherPos));

                if (!dopplerValue.InRange(minDop, maxDop))
                {
                    // Store the Doppler shift value in the result array
                    result[0] = dopplerValue;
                }
                else
                {
                    // If the Doppler shift is not within the range, store a default value
                    result[0] = float.MaxValue;
                }
            }
        }

        public static float DopplerValue(Rigidbody thisThing, Rigidbody otherThing, float minDop, float maxDop)
            {
                NativeArray<float> result = new NativeArray<float>(64, Allocator.TempJob);
                Vector3 thisVel = thisThing.velocity;
                Vector3 otherVel = otherThing.velocity;
                Vector3 thisPos = thisThing.transform.position;
                Vector3 otherPos = otherThing.transform.position;

                DopplerJob dopplerJob = new DopplerJob
                {
                    thisRBVel = thisVel,
                    otherRBVel = otherVel,
                    thisPos = thisPos,
                    otherPos = otherPos,
                    result = result
                };

                JobHandle handle = dopplerJob.Schedule();
                handle.Complete();

                float dopplerValue = result[0];

                result.Dispose();

                return dopplerValue;
            }

        /// <summary>
        /// Gets whatever is bearing down most aggressively towards the current transform
        /// </summary>
        /// <param name="rList"></param>
        /// <param name="thisRB"></param>
        /// <param name="dMin"></param>
        /// <param name="dMax"></param>
        /// <returns></returns>
        public static T BearingDownOnThis<T,O>(this O thisRB, List<T> rList) where T : MonoPlus where O : Component
        {
            float minVal = 0f;
            T t = null;

            foreach (T RR in rList)
            {
                // Add the radiation returns alongside the angle between their bearing and the original transform.
                float f = RR.transform.position.DirTo(thisRB.transform.position).AngleTo(RR.rigidBody.velocity);

                if (f < minVal)
                {
                    minVal = f;
                    t = RR;
                }
            }

            //Get whatever object is currently heading towards this point in space more than all others.
            return t;
        }
        public struct AngleCalculationJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<Vector3> velocities;
            [ReadOnly] public NativeArray<Vector3> positions;
            public NativeArray<float> angles;
            public Vector3 point;

            public void Execute(int index)
            {
                angles[index] = positions[index].DirTo(point).AngleTo(velocities[index]);
            }
        }

        public static List<T> SortByBearing<T>(this List<T> components, Vector3 point) where T : MonoPlus
        {
            int count = components.Count;
            if (count == 0) return components;

            NativeArray<Vector3> velocities = new NativeArray<Vector3>(count, Allocator.TempJob);
            NativeArray<Vector3> positions = new NativeArray<Vector3>(count, Allocator.TempJob);
            NativeArray<float> angles = new NativeArray<float>(count, Allocator.TempJob);

            // Copy positions and velocities to their respective NativeArrays
            for (int i = 0; i < count; i++)
            {
                Rigidbody otherRB = components[i].rigidBody;

                if (otherRB)
                {
                    velocities[i] = otherRB.velocity.normalized;
                    positions[i] = components[i].transform.position;
                }
            }

            // Schedule job to compute distances in parallel
            AngleCalculationJob job = new AngleCalculationJob
            {
                velocities = velocities,
                positions = positions,
                point = point,
                angles = angles
            };
            JobHandle handle = job.Schedule(count, 64);

            // Wait for job to complete
            handle.Complete();

            // Create a copy of the original list
            List<T> sortedComponents = new List<T>(components);

            sortedComponents.ToArray().SortAsync(angles.ToArray());

            // Cleanup
            velocities.Dispose();
            positions.Dispose();
            angles.Dispose();

            return sortedComponents;
        }

        public static void PlaySoundAtPoint(this MonoBehaviour MB, Vector3 point, AudioClip A, float volume)
        {
            GameObject GO = new GameObject();
            AudioSource AS = GO.AddComponent<AudioSource>();

            GO.transform.position = point;
            AS.volume = volume;

            AS.PlayOneShot(A);

            MB.StartCoroutine(SoundDisposer(AS));
        }

        public static void PlaySoundAtPoint(this MonoBehaviour MB, Vector3 point, AudioClip A, float volume, out AudioSource AS)
        {
            GameObject GO = new GameObject();
            AudioSource ASource = GO.AddComponent<AudioSource>();

            GO.transform.position = point;
            ASource.volume = volume;

            ASource.PlayOneShot(A);

            MB.StartCoroutine(SoundDisposer(ASource));

            AS = ASource;
        }
        /*
        public static IEnumerator LoopSoundAtPoint(this MonoBehaviour MB, Vector3 point, AudioClip A, float volume)
        {
            GameObject GO = new GameObject();
            AudioSource AS = GO.AddComponent<AudioSource>();

            GO.transform.position = point;
            AS.volume = volume;

            AS.PlayOneShot(A);

            MB.StartCoroutine(SoundDisposer(AS));
        }

        public static IEnumerator LoopSoundAtPoint(this MonoBehaviour MB, Vector3 point, AudioClip A, float volume, out AudioSource AS)
        {
            GameObject GO = new GameObject();
            AudioSource ASource = GO.AddComponent<AudioSource>();

            GO.transform.position = point;
            ASource.volume = volume;

            ASource.PlayOneShot(A);

            MB.StartCoroutine(SoundDisposer(ASource));

            AS = ASource;
        }*/

        public static IEnumerator SoundDisposer(AudioSource AS)
        {
            //Wait until the audioSource is no longer playing.
            yield return new WaitUntil(() => !AS.isPlaying);

            //Then, once it is not, destroy it.
            MonoBehaviour.Destroy(AS.gameObject);
        }

        public static AudioClip GenerateSineWaveTone(float duration, float frequency)
        {
            int sampleRate = AudioSettings.outputSampleRate;
            int numSamples = (int)(duration * sampleRate);
            float[] samples = new float[numSamples];

            float increment = frequency * 2 * Mathf.PI / sampleRate;
            float angle = 0f;

            for (int i = 0; i < numSamples; i++)
            {
                samples[i] = Mathf.Sin(angle);
                angle += increment;
            }

            return AudioClip.Create("SineWaveTone", numSamples, 1, sampleRate, false);
        }
    }

    public static class AirWorks
    {
        // Constants for ISA model
        private const float R = 287.058f; // Specific gas constant for dry air (J/kg�K)
        private const float T0 = 288.15f; // Standard temperature at sea level (K)
        private const float p0 = 101325f; // Standard pressure at sea level (Pa)

        public struct AirDensityJob : IJob
        {
            public float T0;
            public float p0;
            public float R;
            public float altitude;
            public NativeArray<float> densities;

            public void Execute()
            {
                float T = T0 - 0.0065f * altitude; // lapse rate -6.5�C/km (Kelvin)
                float p = p0 * Mathf.Pow((1f - 0.0065f / T0 * altitude / 288.15f), 5.2561f); // (Pascal)
                float rho = p / (R * T); // (kg/m^3)

                densities[0] = rho;
            }
        }
        public static float AirDensity(this Transform transform)
        {
            NativeArray<float> densities = new NativeArray<float>(1, Allocator.TempJob);

            AirDensityJob job = new AirDensityJob
            {
                T0 = T0,
                p0 = p0,
                R = R,
                altitude = transform.Altitude(),
                densities = densities
            };

            JobHandle jobHandle = job.Schedule();
            jobHandle.Complete();

            float density = densities[0];
            densities.Dispose();

            return density;
        }

        public static float Altitude(this Transform t)
        {
            return t.position.y;
        }

        public static float DragCalc(this Rigidbody rb, float dragCoefficient, float referenceArea)
        {
            //Calculate air density based on the transform of the Rigidbody
            float airDensity = rb.transform.AirDensity();

            // Calculate the angle between the forward direction and the velocity vector
            float angleVelFwrd = rb.transform.forward.AngleTo(rb.velocity.normalized);

            // Calculate drag based on the angle (assuming linear relationship)
            float drag = Mathf.Abs(angleVelFwrd - 90f) / 90f;

            // Calculate drag force
            float dragForce = -0.5f * drag * dragCoefficient * referenceArea * airDensity;

            return dragForce;
        }
    }
    public static class PhysWorks
        {
        // Job to calculate minimum distance between two lines
        struct MinDistBetweenLinesJob : IJob
        {
            public Vector3 line1Start;
            public Vector3 line1End;
            public Vector3 line2Start;
            public Vector3 line2End;
            public NativeArray<float> result;

            public void Execute()
            {
                Vector3 closestPointOnLine1;
                Vector3 closestPointOnLine2;

                // Direction vectors of the lines
                Vector3 line1Direction = line1End - line1Start;
                Vector3 line2Direction = line2End - line2Start;

                // Vectors between the start points of the lines
                Vector3 start1ToStart2 = line2Start - line1Start;

                // Calculate parameters to represent the lines in terms of t (line parameter)
                float a = Vector3.Dot(line1Direction, line1Direction);
                float b = Vector3.Dot(line1Direction, line2Direction);
                float c = Vector3.Dot(line2Direction, line2Direction);
                float d = Vector3.Dot(line1Direction, start1ToStart2);
                float e = Vector3.Dot(line2Direction, start1ToStart2);

                // Calculate the parameter values for the closest points on each line
                float denom = a * c - b * b;
                float t1 = Mathf.Clamp01((b * e - c * d) / denom);
                float t2 = Mathf.Clamp01((a * e - b * d) / denom);

                // Calculate the closest points on each line using the parameter values
                closestPointOnLine1 = line1Start + t1 * line1Direction;
                closestPointOnLine2 = line2Start + t2 * line2Direction;

                // Store the distance between the closest points
                result[0] = closestPointOnLine1.DistTo(closestPointOnLine2);
            }
        }

        // Calculate the minimum distance between two lines using Unity Jobs
        public static bool DistancePassTest(Rigidbody rb1, Rigidbody rb2, float minDist)
        {
            // Get the start and end points of the lines using Rigidbody positions
            Vector3 line1Start = rb1.position;
            Vector3 line1End = rb1.position + rb1.velocity;
            Vector3 line2Start = rb2.position;
            Vector3 line2End = rb2.position + rb2.velocity;

            // Create NativeArray to store result
            NativeArray<float> result = new NativeArray<float>(1, Allocator.TempJob);

            // Create job instance
            MinDistBetweenLinesJob job = new MinDistBetweenLinesJob()
            {
                line1Start = line1Start,
                line1End = line1End,
                line2Start = line2Start,
                line2End = line2End,
                result = result
            };

            // Schedule the job
            JobHandle handle = job.Schedule();

            // Wait for the job to complete
            handle.Complete();

            // Get the result from the NativeArray
            float distance = result[0];

            // Dispose the NativeArray
            result.Dispose();

            // Is the minimum distance between the velocities of the rigidbodies less than the minimum distance specified?
            return distance < minDist;
        }
        /*
        // Calculate the minimum distance between two lines using Rigidbody velocity vectors
        public static bool DistancePassTest(Rigidbody rb1, Rigidbody rb2, float minDist)
            {
                // Get the start and end points of the lines using Rigidbody positions
                Vector3 line1Start = rb1.position;
                Vector3 line1End = rb1.position + rb1.velocity;
                Vector3 line2Start = rb2.position;
                Vector3 line2End = rb2.position + rb2.velocity;

                // Is the minimum distance between the velocities of the rigidbodies going to be less than the minimum distance specified?
                return (MinDistBetweenLines(line1Start, line1End, line2Start, line2End) < minDist);
            }

            // Calculate the minimum distance between two lines
            public static float MinDistBetweenLines(Vector3 line1Start, Vector3 line1End, Vector3 line2Start, Vector3 line2End)
            {
                Vector3 closestPointOnLine1;
                Vector3 closestPointOnLine2;

                // Direction vectors of the lines
                Vector3 line1Direction = line1End - line1Start;
                Vector3 line2Direction = line2End - line2Start;

                // Vectors between the start points of the lines
                Vector3 start1ToStart2 = line2Start - line1Start;

                // Calculate parameters to represent the lines in terms of t (line parameter)
                float a = Vector3.Dot(line1Direction, line1Direction);
                float b = Vector3.Dot(line1Direction, line2Direction);
                float c = Vector3.Dot(line2Direction, line2Direction);
                float d = Vector3.Dot(line1Direction, start1ToStart2);
                float e = Vector3.Dot(line2Direction, start1ToStart2);

                // Calculate the parameter values for the closest points on each line
                float denom = a * c - b * b;
                float t1 = Mathf.Clamp01((b * e - c * d) / denom);
                float t2 = Mathf.Clamp01((a * e - b * d) / denom);

                // Calculate the closest points on each line using the parameter values
                closestPointOnLine1 = line1Start + t1 * line1Direction;
                closestPointOnLine2 = line2Start + t2 * line2Direction;

                // Calculate and return the distance between the closest points
                return Vector3.Distance(closestPointOnLine1, closestPointOnLine2);
            }
        */
    }

    public class MR : MonoBehaviour
    {
        public static MR mr = null;

        private void Awake()
        {
            mr = this;
        }
    }
}