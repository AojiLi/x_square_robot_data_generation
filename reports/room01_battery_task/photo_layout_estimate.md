# 2026-09-16 real battery insertion scene: photo estimate

This is a **visual reconstruction estimate**, not camera calibration or a measured robot-to-table transform. The real robot is absent from these photos, so its table-relative pose cannot be recovered from them. Dimensions explicitly supplied by the user take precedence over image estimates.

## Coordinates and measured dimensions

Table-centred coordinates: x is right in the photos, y points toward the rear wall, z points upward. The tabletop is z = 0 locally and z = 0.740 m above the floor.

| Object | User-supplied dimensions in metres |
| --- | --- |
| White table | x = 1.100; y = 0.570; tabletop floor height = 0.740 |
| Each yellow sponge | x = 0.080; y = 0.098; z = 0.052 |
| Each battery | diameter = 0.0125; axial length = 0.049 |
| Red foam block | x = 0.072; y = 0.048; z = 0.048 |
| Each red-block hole, latest user confirmation | diameter = 0.0125; depth = 0.013 |
| Table opening | total length = 0.230; straight length = 0.153; rear edge to table rear edge = 0.092 |

The x/y orientation assigned to sponge dimensions follows the photos: yellow blocks are longer front-to-back; red block is longer left-to-right. The user subsequently confirmed that the battery's 1.25 cm width is its cylindrical diameter, that the receiving holes have the same 1.25 cm diameter, and that their depth is 1.3 cm. These confirmations supersede photo-only estimates. Hole pitch remains image-estimated.

## Recommended initial placement

| Object | Local x, y in metres | Rotation of long/axial direction |
| --- | --- | --- |
| Left yellow sponge | (-0.270, -0.068) | Long sponge edge approximately +y |
| Right yellow sponge | (+0.215, -0.067) | Long sponge edge approximately +y |
| Red three-hole block | (-0.049, -0.083) | Long red edge approximately +x |
| Left battery | (-0.273, -0.059) | +60 degrees relative to +x |
| Right battery | (+0.203, -0.064) | +20 degrees relative to +x |

Use about **1–2 cm placement uncertainty** and about **5–10 degrees battery yaw uncertainty**. The table-relative source objects are repeatably asymmetric: the left sponge is farther from table centre than the right sponge, and the red block lies about 5 cm left of table centre. The red block is about 1–2 cm in front of the sponge centres. The right battery is about 1 cm left of its sponge centre. Battery axial direction has a 180-degree ambiguity: polarity is not established by these photos.

If the battery rests tangentially on a flat undeformed sponge, its axis height is 0.052 + 0.0125/2 = 0.05825 m above the tabletop (0.79825 m above the floor). Real foam deformation and resting groove depth have not been measured.

## Perspective and height correction evidence

Manually estimated intersections of the four straight table edges were used rather than the visibly rounded silhouette corners. Table-plane homographies were fitted to the 1.100 x 0.570 m rectangle. For a height correction, a simple pinhole camera was fitted with square pixels, zero skew and principal point at image centre, using least-squares rotation, translation and focal length. Rays were intersected at sponge top height 0.052 m or red block top height 0.048 m. Lens distortion was not modelled; nominal subpixel/2-pixel reprojection residuals below are fitting residuals, **not evidence of metric accuracy**.

Table corners are listed rear-left, rear-right, front-right, front-left, in image pixels:

| Image | Original pixels | Table edge-intersection picks | Approximate fitted focal length | Fitted corner RMS |
| --- | --- | --- | --- | --- |
| #10 be08afecb4601d1fb2179457fc3dfba7.jpg | 1707 x 1280 | (318,424); (1341,383); (1572,838); (63,865) | 1151 px | 1.0 px |
| #11 a1bfb02711b58056bb4c0052031952ee.jpg | 1280 x 960 | (278,410); (1035,405); (1159,859); (99,802) | 807 px | 1.8 px |
| #7 126401418fdd670729f962714a4ca846.jpg | 1280 x 960 | (421,470); (1070,471); (1218,781); (272,746) | 855 px | 2.5 px |

Hand-selected object top-centre image pixels:

| Image | Left sponge | Red block | Right sponge | Opening centre |
| --- | --- | --- | --- | --- |
| #10 | (478,606) | (742,615) | (1060,587) | (816,473) |
| #11 | (378,605) | (563,624) | (798,612) | (634,473) |
| #7 | (517,596) | (674,607) | (879,602) | (720,515) |

Recovered x/y in metres after the height correction:

| Image | Left sponge | Red block | Right sponge |
| --- | --- | --- | --- |
| #10 | (-0.2707,-0.0698) | (-0.0477,-0.0858) | (+0.2129,-0.0680) |
| #11 | (-0.2680,-0.0639) | (-0.0443,-0.0804) | (+0.2189,-0.0606) |
| #7 | (-0.2728,-0.0704) | (-0.0541,-0.0825) | (+0.2125,-0.0714) |

Without correcting the height, the same sponge-top pixels would suggest x about -0.30 / +0.22 and y about -0.01 to -0.04; this is why direct table homography projection of raised object centres should not be used as ground truth.

Battery directions were independently estimated using visible axial point pairs, then projected to z = 0.05825 m. The pairs follow the visible blue body and are not exact cap-to-cap endpoints:

| Image | Left pixel pair; fitted angle | Right pixel pair; fitted angle |
| --- | --- | --- |
| #10 | (460,608) to (491,580); 61.4 degrees | (1022,585) to (1068,571); 21.2 degrees |
| #11 | (363,613) to (388,586); 60.0 degrees | (773,615) to (804,605); 21.0 degrees |
| #7 | (507,593) to (527,576); 58.7 degrees | (852,600) to (883,593); 18.6 degrees |

## Opening and insertion geometry

The tabletop opening is approximately centred in x. Independent fits give x near 0.0005, 0.012 and -0.0045 m. Use x = 0 initially, with roughly 1 cm visual uncertainty.

If the two curved ends are semicircles and the measured straight length describes the rectangular part, then the opening width is derived as 0.230 - 0.153 = **0.077 m** and the end radius is **0.0385 m**. Its measured rear margin gives the local centre y = 0.285 - 0.092 - 0.077/2 = **+0.1545 m**. This derivation is consistent with the visible capsule shape. The opening's image-estimated y centres are +0.164 to +0.169 m, within roughly 1–1.5 cm of that derived value; the supplied dimensions should win. The photos show a pale rim around a real opening, not a dark decal. Rim outer versus opening inner dimensions were not separately specified.

The red block has three visibly open circular mouths in a left-to-right row. The original photo-only estimate of their diameter, from the 72 mm block width, was approximately **12 mm**, plausibly **10–14 mm** at this image resolution. The latest user confirmation supplies the authoritative hole diameter **12.5 mm** and hole depth **13 mm**. At a 48 mm block height this defines a blind bore whose bottom lies 35 mm above the tabletop. The same 12.5 mm nominal diameter for battery and bore gives zero nominal diametral clearance; foam deformation and manufacturing tolerance are not measured by these dimensions.

The centre spacing remains approximately **20 mm**, plausibly **18–22 mm**, estimated from the photos. A centred provisional row at local block x = (-0.020, 0, +0.020) m is reasonable. Hole taper and foam compliance remain unmeasured. The confirmed depth and diameter must not be labelled as unresolved photo estimates.

The two light strips on the front of the red block appear to be tape adhered to its front face and continuing onto the tabletop. They should be represented as surface tape details, **not two cut-out legs or slots**. Additional translucent tape is visible near the block's sides. The images suggest the red block is secured, but actual fixture rigidity is not measured.

## Visible scene changes and limits

The manipulation set is now two yellow foam supports, each carrying one horizontal blue cylindrical battery, and one smaller red three-hole receiving block between them. The receiver is burgundy/dark red, the supports bright yellow, the cylinders turquoise/blue with end-cap detail. The existing room still shows the white work table next to the dark optical breadboard and monitor; the wall, carpet, table legs, computer under the table, and blue/black cables are background context.

### Relocation beside the stationary computer workstation

**User correction:** the robot and white table were moved beside the computer. The computer desk, monitor and PC tower stay at their original room coordinates. The earlier interpretation that moved the computer workstation was incorrect and has been replaced.

The photos place the optical breadboard to the left of the white table, with their rear edges approximately aligned. Using the existing modeled optical-table bounds x = [1.280, 2.045], y = [-0.346, 0.846] m, the relocated white table has center **[1.760, -0.910, 0] m**, bounds x = [1.475, 2.045], y = [-1.460, -0.360] m, and tabletop z = 0.740 m. The table and robot receive the same world translation **[+0.7255, +1.027, 0] m**, retaining the robot-to-table transform. The robot base becomes **[0.9455, -0.910, 0.002] m**. Table props and robot-mounted cameras follow their respective parents.

The 14 mm side gap to the computer desk and 3 mm gap to the modeled right return wall are consequences of this approximate placement, not measurements from the photographs. After accounting for different table heights, the photograph permits several centimetres of rear-edge mismatch; no millimetre-accurate absolute room registration is claimed. The PC tower is now under the left side of the white table, consistent with the photographs.

The old model's headset intersected the relocated table and is not visible in the supplied photos, so it is hidden in this task layer. The floor powerstrip would occupy the new robot footprint; it is placed below the table according to the visible under-table power arrangement, with its exact position explicitly estimated. The desk, monitor, keyboard and PC tower are not moved.
