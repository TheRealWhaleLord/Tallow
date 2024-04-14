using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Threading.Tasks;

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

        public static Vector3 DirTo<T,O>(this T firstThing, O otherThing) where T : Component where O : Component
        {
            // Calculate the direction to the player
            return (otherThing.transform.position - firstThing.transform.position).normalized;
        }

        public static Vector3 DirTo(this Vector3 firstThing, Vector3 otherThing)
        {
            // Calculate the direction to the player
            return (otherThing - firstThing).normalized;
        }

        public static Vector3 CalculateCenter(this List<Vector3> points)
        {
            //If there is only one item in the list of Vectors, give it back as the center.
            if (points.Count == 1) return points[0];

            Vector3 center = Vector3.zero;

            // Sum all points
            foreach (Vector3 point in points)
            {
                center += point;
            }

            // Divide by the number of points to get the average
            center /= points.Count;

            return center;
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

        public static float DistTo<T, O>(this T to, O distToThis) where T : Component where O : Component
        {
            return Vector3.Distance(to.transform.position, distToThis.transform.position);
        }

        public static float DistTo<T>(this T to, Vector3 distToThis) where T : Component
        {
            return Vector3.Distance(to.transform.position, distToThis);
        }

        public static float DistTo(this Vector3 to, Vector3 distToThis)
        {
            return Vector3.Distance(to, distToThis);
        }

        public static void SafeAdd<T>(this List<T> toThis, T addThis)
        {
            if (!toThis.Contains(addThis)) toThis.Add(addThis);
        }

        public static void SafeRemove<T>(this List<T> fromThis, T removeThis)
        {
            if (fromThis.Contains(removeThis)) fromThis.Remove(removeThis);
        }

        public static Dictionary<TKey, TValue> Sort<TKey, TValue>(this Dictionary<TKey, TValue> dictionary)
        {
            return dictionary.OrderBy(x => x.Value).ToDictionary(pair => pair.Key, pair => pair.Value);
        }

        public static bool Empty<T>(this List<T> list)
        {
            if (list.Count == 0) return true;
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

        public static T AddOrGet<T>(this GameObject go) where T : Component
        {
            if (go.TryGetComponent<T>(out T component))
                return component;
            else return go.AddComponent<T>();
        }

        public static float RandRange(float first, float second)
        {
            return UnityEngine.Random.Range(first, second);
        }
        public static int RandRange(int first, int second)
        {
            return UnityEngine.Random.Range(first, second);
        }

        public static IEnumerator DestroyAfter<T>(this T wipeThis, float afterThisTime) where T : Component
        {
            //Wait until the timer has depleted entirely before continuing.
            while (afterThisTime > 0 && wipeThis)
            {
                afterThisTime -= Time.deltaTime;
                yield return null;
            }

            if (wipeThis) MonoBehaviour.Destroy(wipeThis.gameObject);
        }

        public static IEnumerator DestroyAfter(this GameObject wipeThis, float afterThisTime)
        {
            //Wait until the timer has depleted entirely before continuing.
            while (afterThisTime > 0 && wipeThis)
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
            while (dist < afterThisDist && wipeThis)
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
        public static GameObject GetClosest(this Vector3 closestTo, List<GameObject> LGO)
        {
            Dictionary<GameObject, float> DistOfGameObjs = new Dictionary<GameObject, float>();

            foreach (GameObject GO in LGO)
            {
                DistOfGameObjs.Add(GO, Vector3.Distance(closestTo, GO.transform.position));
            }

            // Sort the dictionary by value of distance, then return the first of these values.
            return DistOfGameObjs.OrderBy(x => x.Value).ToDictionary(pair => pair.Key, pair => pair.Value).Keys.First();
        }

        public static float AngleTo(this Vector3 thing, Vector3 otherThing)
        {
            return Mathf.Abs(Vector3.Angle(thing, otherThing));
        }

        /// <summary>
        /// Returns the closest component, by distance, to the conponent specified fron the list specified.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <typeparam name="O"></typeparam>
        /// <param name="LGO"></param>
        /// <param name="closestTo"></param>
        /// <returns></returns>
        public static T ClosestTo<T, O>(this List<T> LGO, O closestTo) where T : Component where O : Component
        {
            Dictionary<T, float> DistOfGameObjs = new Dictionary<T, float>();

            foreach (T GO in LGO)
            {
                DistOfGameObjs.Add(GO, UnityEngine.Vector3.Distance(closestTo.transform.position, GO.transform.position));
            }

            // Sort the dictionary by value of distance, then return the first of these values.
            return DistOfGameObjs.OrderBy(x => x.Value).ToDictionary(pair => pair.Key, pair => pair.Value).Keys.ToList().First();
        }

        public static T ClosestTo<T>(this List<T> LGO, Vector3 closestTo) where T : Component
        {
            Dictionary<T, float> DistOfGameObjs = new Dictionary<T, float>();

            foreach (T GO in LGO)
            {
                DistOfGameObjs.Add(GO, Vector3.Distance(closestTo, GO.transform.position));
            }

            // Sort the dictionary by value of distance, then return the first of these values.
            return DistOfGameObjs.OrderBy(x => x.Value).ToDictionary(pair => pair.Key, pair => pair.Value).Keys.ToList().First();
        }

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
                distances.Add(item, UnityEngine.Vector3.Distance(getPosition(closestTo), getPosition(item)));
            }

            // Sort the dictionary by distance and return the sorted keys
            return distances.OrderBy(x => x.Value).Select(x => x.Key).ToList();
        }

        public static List<T> SortByDistance<T>(this Vector3 closestTo, List<T> LGO) where T : Component
        {
            Dictionary<T, float> DistOfGameObjs = new Dictionary<T, float>();

            foreach (T GO in LGO)
            {
                DistOfGameObjs.Add(GO, Vector3.Distance(closestTo, GO.transform.position));
            }

            // Sort the dictionary by value of distance, then return the first of these values.
            return DistOfGameObjs.OrderBy(x => x.Value).ToDictionary(pair => pair.Key, pair => pair.Value).Keys.ToList();
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
                else
                {
                    break; // Exit the loop if we reached the end of the input list
                }
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

        public static List<RadReturn> DopplerFilter(this List<RadReturn> rList, Rigidbody thisRB, float dMin, float dMax)
        {
            Dictionary<RadReturn, float> RadRets = new Dictionary<RadReturn, float>();

            foreach (RadReturn RR in rList)
            {
                if (RR.TryGetComponent<Rigidbody>(out Rigidbody otherRB))
                {
                    float doppler = DopplerValue(thisRB, otherRB);

                    if (!doppler.InRange(dMin, dMax))
                    {
                        RadRets.Add(RR, doppler);
                        //Debug.Log(RR + " was added with doppler value " + doppler + " between " + dMin + " and " + dMax);
                    }
                }
            }

            // Return the keys (RadReturn objects) in the sorted dictionary as a list, from the greatest DopplerValues to the smallest (which are likely to be chaff).
            return RadRets.OrderByDescending(kv => kv.Value).ToDictionary(pair => pair.Key, pair => pair.Value).Keys.ToList();
        }

        public static float DopplerValue(Rigidbody thisThing, Rigidbody otherThing)
        {
            Vector3 thisRBVel = Vector3.zero;
            Vector3 otherRBVel = Vector3.zero;

            if (thisThing) thisRBVel = thisThing.velocity;
            if (otherThing) otherRBVel = otherThing.velocity;

            // Calculate the relative velocity between the radar return and the object
            Vector3 relativeVelocity = otherRBVel - thisRBVel;

            // Calculate the Doppler effect based on the relative velocity
            return Vector3.Dot(relativeVelocity, (thisThing.position - otherThing.position).normalized);
        }

        /// <summary>
        /// Summarizes and filters based on the heading and bearing of the targets in relation to the original rigidbody.
        /// Sorted by what targets are currently moving towards the calling rigidbody.
        /// </summary>
        /// <param name="rList"></param>
        /// <param name="thisRB"></param>
        /// <param name="dMin"></param>
        /// <param name="dMax"></param>
        /// <returns></returns>
        public static List<RadReturn> HeadingFilter(this List<RadReturn> rList, Transform thisRB)
        {
            Dictionary<RadReturn, float> rigidBodies = new Dictionary<RadReturn, float>();

            foreach (RadReturn RR in rList)
            {
                if (RR.TryGetComponent<Rigidbody>(out Rigidbody otherRB))
                {
                    // Add the radiation returns alongside the angle between their bearing and the original transform.
                    rigidBodies.Add(RR, RR.transform.position.DirTo(thisRB.transform.position).AngleTo(otherRB.velocity));
                }
            }

            // Return the keys in the sorted dictionary as a list, from the least to the greatest.
            return rigidBodies.OrderBy(kv => kv.Value).ToDictionary(pair => pair.Key, pair => pair.Value).Keys.ToList();
        }

        public static List<Rigidbody> HeadingFilter(this List<Rigidbody> rList, Transform thisRB)
        {
            Dictionary<Rigidbody, float> rigidBodies = new Dictionary<Rigidbody, float>();

            foreach (Rigidbody RR in rList)
            {
                // Add the radiation returns alongside the angle between their bearing and the original transform.
                rigidBodies.Add(RR, RR.transform.position.DirTo(thisRB.transform.position).AngleTo(RR.velocity));
            }

            // Return the keys in the sorted dictionary as a list, from the least to the greatest.
            return rigidBodies.OrderBy(kv => kv.Value).ToDictionary(pair => pair.Key, pair => pair.Value).Keys.ToList();
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
        private const float R = 287.058f; // Specific gas constant for dry air (J/kg·K)
        private const float T0 = 288.15f; // Standard temperature at sea level (K)
        private const float p0 = 101325f; // Standard pressure at sea level (Pa)

        public static float AirDensity(this Transform t)
        {
            //Get the altitude of the transform.
            float altitude = t.Altitude();

            // Calculate temperature at altitude (ISA model)
            float T = T0 - 0.0065f * altitude; // lapse rate -6.5°C/km (Kelvin)

            // Calculate pressure at altitude (ISA model)
            float p = p0 * Mathf.Pow((1f - 0.0065f / T0 * altitude / 288.15f), 5.2561f); // (Pascal)

            // Calculate air density using the ideal gas law (Pascal)
            float rho = p / (R * T); // (kg/m^3)

            return rho;
        }

        public static float Altitude(this Transform t)
        {
            return t.position.x;
        }

        public static float DragCalc(this Rigidbody rb, float dragCoefficient, float referenceArea)
        {
            //Calculate air density based on the transform of the Rigidbody
            float airDensity = rb.transform.AirDensity();

            // Calculate the angle between the forward direction and the velocity vector
            float angleVelFwrd = Vector3.Angle(rb.transform.forward, rb.velocity.normalized);

            // Calculate drag based on the angle (assuming linear relationship)
            float drag = Mathf.Abs(angleVelFwrd - 90f) / 90f;

            // Calculate drag force
            float dragForce = -0.5f * drag * dragCoefficient * referenceArea * airDensity;

            return dragForce;
        }
    }
    public static class PhysWorks
        {
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