scaled = 2  # defines by which factor the image is subsampled
width = camera.getWidth()
height = camera.getHeight()

# read rgb pixel values from the camera
image = camera.getImage()

print '----------camera image (gray levels)---------'
print 'original resolution: %d x %d, scaled to %d x %f' \
      % (width, height, width / scaled, height / scaled)
outer = []
for y in range(0, height / scaled):
    inner = []
    for x in range(0, width / scaled):
        gray = camera.imageGetGray(image, width, x * scaled,
                                   y * scaled) * 9 / 255  # between 0 and  instead of 0 and 255
        inner.append(gray)
    outer.append(gray)