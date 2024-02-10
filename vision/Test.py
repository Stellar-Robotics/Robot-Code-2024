import robotpy_apriltag as apriltag

def main():
    field = apriltag.AprilTagField.k2024Crescendo
    fieldLayout = apriltag.loadAprilTagLayoutField(field=field)
    fieldLayout.serialize("./fieldLayout.json")
    print(fieldLayout.getTagPose(16))

if __name__ == "__main__":
   main()