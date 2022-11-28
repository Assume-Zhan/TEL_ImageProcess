# Block detector
> For TEL block detecting

## Preprocessor
> For preprocessing the image before capture the image

### Blur the image
> Gaussian blur
- Kernel size (3, 3)
- x-direction standard deviation 0
- Bolder type : BORDER_REFLECT
```cpp=1
cv::GaussianBlur(frame, frame, cv::Size(9, 9), 3, 0);
```

### Color filter
> Mask with light and dark mask
- Parameter can be modify in launch file

### Find the bolder
> Canny 
- Canny function

### Threshold


---

## Block catcher
> Use the 1 0 frame after preprocessor to detect the position of the block