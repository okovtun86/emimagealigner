## emimagealigner
A lightweight MATLAB app (single .m class) for aligning EM images with manual landmarks and fast keyboard-based affine tweaks (translate/rotate/scale/shear)

### Features
- Manual landmark picking via cpselect (robust RANSAC optional)
- Models: translation, rigid, similarity, affine, projective, polynomial2, pwl, lwm (tested in MATLAB R2022b and R2025a)
- Real-time overlay with alpha slider
- Keyboard Nudge mode for smooth corrections 
- Saves warped result to PNG/TIFF/JPEG

### Install

1. Copy ImageAlignerApp.m into your MATLAB path
2. Open MATLAB and run:
   ```
   app = ImageAlignerApp;
   ```

### Usage

1. Load Fixed (FIB-SEM) and Load Moving (SEM) images
2. Pick Landmarks → select points in cpselect → Close cpselect window
3. Nudge: Enable to make fine adjustments via keyboard
4. Nudge: Commit (Enter) to bake in, or Nudge: Cancel (Esc) to discard
5. Save Result to export the warped moving image

### Keyboard Nudge Controls

| Action      | Keys |
|---|---|
| Translate   | <kbd>←</kbd> <kbd>→</kbd> <kbd>↑</kbd> <kbd>↓</kbd>  (hold <kbd>Shift</kbd> for 10×) |
| Rotate      | <kbd>&lt;</kbd> <kbd>&gt;</kbd> |
| Scale X     | <kbd>9</kbd>/<kbd>-</kbd> (down), <kbd>0</kbd>/<kbd>=</kbd> (up) |
| Scale Y     | <kbd>;</kbd> (down), <kbd>'</kbd> (up) |
| Shear X     | <kbd>1</kbd> (−), <kbd>2</kbd> (+) |
| Shear Y     | <kbd>3</kbd> (−), <kbd>4</kbd> (+) |
| Commit      | <kbd>Enter</kbd> |
| Cancel      | <kbd>Esc</kbd> |


### License: MIT

### Citation
If this aids your EM alignment workflow, please cite this repo in your methods or acknowledgments
