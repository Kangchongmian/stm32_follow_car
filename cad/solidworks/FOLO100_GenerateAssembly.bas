Attribute VB_Name = "FOLO100_GenerateAssembly"
Option Explicit

' FOLO-100 SolidWorks assembly generator
' Run this VBA macro inside SolidWorks. It creates simplified SLDPRT files and
' a positioned FOLO100_Automatic_Follow_Cart.SLDASM assembly under:
'   %USERPROFILE%\Documents\FOLO100_SolidWorks
'
' Units in this macro are millimetres at the design level and converted to
' metres for the SolidWorks API.

Private Const MM As Double = 0.001
Private Const swDocPART As Long = 1
Private Const swDocASSEMBLY As Long = 2
Private Const swSaveAsOptions_Silent As Long = 1
Private Const swEndCondBlind As Long = 0
Private Const swEndCondMidPlane As Long = 6

Private swApp As Object
Private outDir As String
Private partsDir As String

Public Sub main()
    Set swApp = Application.SldWorks

    outDir = Environ$("USERPROFILE") & "\Documents\FOLO100_SolidWorks"
    partsDir = outDir & "\parts"
    EnsureFolder outDir
    EnsureFolder partsDir

    ' Structural parts based on the FOLO-100 reference drawing.
    CreateBoxPart partsDir & "\FOLO100_platform_700x540x18.SLDPRT", 700, 540, 18, 0.74, 0.86, 0.96
    CreateBoxPart partsDir & "\FOLO100_chassis_frame_760x520x40.SLDPRT", 760, 520, 40, 0.78, 0.78, 0.78
    CreateBoxPart partsDir & "\FOLO100_battery_tray_360x190x40.SLDPRT", 360, 190, 40, 0.70, 0.55, 0.86
    CreateBoxPart partsDir & "\FOLO100_control_box_260x180x90.SLDPRT", 260, 180, 90, 0.55, 0.74, 0.94
    CreateBoxPart partsDir & "\FOLO100_motor_plate_180x140x6.SLDPRT", 180, 140, 6, 0.95, 0.74, 0.49
    CreateBoxPart partsDir & "\FOLO100_front_bumper_600x30x35.SLDPRT", 600, 30, 35, 1#, 0.86, 0.30
    CreateBoxPart partsDir & "\FOLO100_side_guard_700x20x210.SLDPRT", 700, 20, 210, 0.86, 0.86, 0.86
    CreateBoxPart partsDir & "\FOLO100_sensor_post_40x40x135.SLDPRT", 40, 40, 135, 0.50, 0.75, 0.50
    CreateBoxPart partsDir & "\FOLO100_lidar_block_90x45x45.SLDPRT", 90, 45, 45, 0.55, 0.85, 0.55

    ' Wheel primitives: cylinders are extruded on the Front plane so their axis
    ' is along vehicle Y when inserted without rotation.
    CreateCylinderYPart partsDir & "\FOLO100_drive_wheel_305x75.SLDPRT", 305, 75, 0.75, 0.75, 0.75
    CreateCylinderYPart partsDir & "\FOLO100_aux_wheel_200x50.SLDPRT", 200, 50, 0.70, 0.55, 0.86

    CreateAssembly

    MsgBox "FOLO-100 SolidWorks assembly generated:" & vbCrLf & _
           outDir & "\FOLO100_Automatic_Follow_Cart.SLDASM", vbInformation
End Sub

Private Sub CreateAssembly()
    Dim swAsm As Object
    Dim errors As Long
    Dim warnings As Long
    Dim asmPath As String

    Set swAsm = swApp.NewAssembly
    asmPath = outDir & "\FOLO100_Automatic_Follow_Cart.SLDASM"

    ' Coordinate convention: X forward, Y left, Z up. API positions are metres.
    AddComponent swAsm, partsDir & "\FOLO100_chassis_frame_760x520x40.SLDPRT", 0, 0, 235
    AddComponent swAsm, partsDir & "\FOLO100_platform_700x540x18.SLDPRT", 0, 0, 405
    AddComponent swAsm, partsDir & "\FOLO100_battery_tray_360x190x40.SLDPRT", 0, 0, 285
    AddComponent swAsm, partsDir & "\FOLO100_control_box_260x180x90.SLDPRT", -240, 0, 300

    AddComponent swAsm, partsDir & "\FOLO100_motor_plate_180x140x6.SLDPRT", 0, 245, 235
    AddComponent swAsm, partsDir & "\FOLO100_motor_plate_180x140x6.SLDPRT", 0, -245, 235
    AddComponent swAsm, partsDir & "\FOLO100_drive_wheel_305x75.SLDPRT", 0, 245, 152.5
    AddComponent swAsm, partsDir & "\FOLO100_drive_wheel_305x75.SLDPRT", 0, -245, 152.5

    AddComponent swAsm, partsDir & "\FOLO100_aux_wheel_200x50.SLDPRT", 300, 210, 100
    AddComponent swAsm, partsDir & "\FOLO100_aux_wheel_200x50.SLDPRT", 300, -210, 100
    AddComponent swAsm, partsDir & "\FOLO100_aux_wheel_200x50.SLDPRT", -300, 210, 100
    AddComponent swAsm, partsDir & "\FOLO100_aux_wheel_200x50.SLDPRT", -300, -210, 100

    AddComponent swAsm, partsDir & "\FOLO100_front_bumper_600x30x35.SLDPRT", 430, 0, 178
    AddComponent swAsm, partsDir & "\FOLO100_front_bumper_600x30x35.SLDPRT", -430, 0, 178
    AddComponent swAsm, partsDir & "\FOLO100_side_guard_700x20x210.SLDPRT", 0, 292, 250
    AddComponent swAsm, partsDir & "\FOLO100_side_guard_700x20x210.SLDPRT", 0, -292, 250

    AddComponent swAsm, partsDir & "\FOLO100_sensor_post_40x40x135.SLDPRT", 365, 0, 472.5
    AddComponent swAsm, partsDir & "\FOLO100_lidar_block_90x45x45.SLDPRT", 390, 0, 520

    swAsm.Extension.SaveAs asmPath, 0, swSaveAsOptions_Silent, Nothing, errors, warnings
End Sub

Private Sub AddComponent(ByVal swAsm As Object, ByVal partPath As String, ByVal xMm As Double, ByVal yMm As Double, ByVal zMm As Double)
    swAsm.AddComponent5 partPath, 0, "", False, "", xMm * MM, yMm * MM, zMm * MM
End Sub

Private Sub CreateBoxPart(ByVal filePath As String, ByVal lengthMm As Double, ByVal widthMm As Double, ByVal heightMm As Double, ByVal red As Double, ByVal green As Double, ByVal blue As Double)
    Dim swModel As Object
    Dim errors As Long
    Dim warnings As Long

    Set swModel = swApp.NewPart
    swModel.Extension.SelectByID2 "Top Plane", "PLANE", 0, 0, 0, False, 0, Nothing, 0
    swModel.SketchManager.InsertSketch True
    swModel.SketchManager.CreateCenterRectangle 0, 0, 0, (lengthMm / 2#) * MM, (widthMm / 2#) * MM, 0
    swModel.SketchManager.InsertSketch True
    swModel.FeatureManager.FeatureExtrusion2 True, False, False, swEndCondMidPlane, swEndCondBlind, heightMm * MM, 0, False, False, False, False, 0, 0, False, False, False, False, True, True, True, 0, 0, False
    ApplyPartColor swModel, red, green, blue
    swModel.Extension.SaveAs filePath, 0, swSaveAsOptions_Silent, Nothing, errors, warnings
    swApp.CloseDoc swModel.GetTitle
End Sub

Private Sub CreateCylinderYPart(ByVal filePath As String, ByVal diameterMm As Double, ByVal widthMm As Double, ByVal red As Double, ByVal green As Double, ByVal blue As Double)
    Dim swModel As Object
    Dim radiusM As Double
    Dim errors As Long
    Dim warnings As Long

    radiusM = (diameterMm / 2#) * MM
    Set swModel = swApp.NewPart
    swModel.Extension.SelectByID2 "Front Plane", "PLANE", 0, 0, 0, False, 0, Nothing, 0
    swModel.SketchManager.InsertSketch True
    swModel.SketchManager.CreateCircleByRadius 0, 0, 0, radiusM
    swModel.SketchManager.InsertSketch True
    swModel.FeatureManager.FeatureExtrusion2 True, False, False, swEndCondMidPlane, swEndCondBlind, widthMm * MM, 0, False, False, False, False, 0, 0, False, False, False, False, True, True, True, 0, 0, False
    ApplyPartColor swModel, red, green, blue
    swModel.Extension.SaveAs filePath, 0, swSaveAsOptions_Silent, Nothing, errors, warnings
    swApp.CloseDoc swModel.GetTitle
End Sub

Private Sub ApplyPartColor(ByVal swModel As Object, ByVal red As Double, ByVal green As Double, ByVal blue As Double)
    Dim materialProps(8) As Double
    materialProps(0) = red
    materialProps(1) = green
    materialProps(2) = blue
    materialProps(3) = 1#
    materialProps(4) = 1#
    materialProps(5) = 0.35
    materialProps(6) = 0.25
    materialProps(7) = 0#
    materialProps(8) = 0#
    swModel.MaterialPropertyValues = materialProps
End Sub

Private Sub EnsureFolder(ByVal folderPath As String)
    If Len(Dir$(folderPath, vbDirectory)) = 0 Then
        MkDir folderPath
    End If
End Sub
