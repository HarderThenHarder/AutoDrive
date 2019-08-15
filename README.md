# This is a project about Lane Line Detection
## Usage
```python
python auto_drive.py
```
In the Window, there are two trackbar:
- Max Drift: This parameter decides the `limitation` of ROI changing. More bigger value you set, ROI change more sharply.
- Close: This parameter defines those points which is `near` the ROI Corners. These close points will also append into the legal_points list. More bigger value you set, more points will be concluded.

## Constructure of this demo
Main porcedure of this demo is: <br>
1. Convert origin frame into HSV, Remain the yellow line part (Others are dropped)
2. Find the edges of frame (binary result)
3. Find the line in the edges frame with `HoughLineP`, get the `point list`.
4. Choose the legal points (which is in ROI or not in ROI but close to the ROI Corners).
5. Calculate the Lane Line with legal point list.
6. Update the ROI

## Calculate Lane Line
Use the legal points to calculate the lane line:<br>
1. Fit the lines in legal lines list (y = kx + b) 
2. Calculate the average value of all lines and get the mean value:k' & b'. Get the average line: y = k'x + b' 

## Update the ROI
This is a important part. I implenment two method of updating, one is using nearest point to update (def updateROIByNearset), and the other is Max Margin Update(def updateROI). The second has a better performance.<br>
### Max Margin Update
The ROI need to be changed in each frame, and it should be as large as it can so that it could conclude more legal points.<br>
1. Calculate the center of ROI.
2. Sort all legal points by their distance to ROI center as Dedcend order.
3. Traverse this sorted list, find the corner which is closet to the legal point.
4. Since the legal point is farest from the ROI center, so let this point be the new ROI corner (if the distance is smaller than Max Drift).