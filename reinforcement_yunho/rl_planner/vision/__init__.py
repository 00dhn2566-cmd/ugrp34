"""Vision in the loop (RESULTS.md §16): Isaac Sim renders the onboard camera, a YOLO-pose detector finds the window
corners, and the same triangulator the policy was trained with (core/perception.py add_detections) turns them into
window estimates — in place of the perception surrogate's synthetic corners."""
