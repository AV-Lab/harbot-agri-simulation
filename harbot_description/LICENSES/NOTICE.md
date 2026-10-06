# Source and license notice

`harbot_description` consolidates retained description files and assets that
previously lived in several ROS packages. Moving those files does not change
their copyright or license.

| Retained source | Version recorded by its former manifest | Declaration available in this repository | New location |
| --- | --- | --- | --- |
| Robotnik `rbkairos_description` | 1.1.2 | BSD; full text in `Robotnik-BSD-3-Clause.txt` | `urdf/base/rbkairos`, `urdf/robots`, `meshes/structures/container` |
| Robotnik `summit_xl_description` | 1.1.3 | BSD; full text in `Robotnik-BSD-3-Clause.txt` | `urdf/base/summit_xl`, `meshes/base/summit_xl` |
| Franka Robotics `franka_description` | 0.10.1 | Apache 2.0 was declared in the former manifest; its full license text was not present in the imported tree | `urdf/arms/franka`, `config/arms/franka`, `meshes/arms/franka` |
| HARBOT soft-finger gripper description | 0.0.0 | The former manifest contained `TODO`; no separate asset license text was present | `urdf/grippers/soft`, `meshes/grippers/soft` |

The HARBOT composition files and repository-authored changes remain covered by
the repository's root MIT license. That MIT license does not replace the
upstream declarations listed above. The missing Franka license text and the
unresolved soft-gripper declaration must be settled before presenting this
package as having a single uniform license.

Recorded upstream locations from the former manifests:

- <https://github.com/RobotnikAutomation/rbkairos_common.git>
- <https://github.com/RobotnikAutomation/summit_xl_common.git>
- <https://github.com/frankarobotics/franka_ros>

Former manifest attribution retained here:

- `rbkairos_description`: Roberto Guzmán and Román Navarro, Robotnik Automation.
- `summit_xl_description`: Roberto Guzmán and Román Navarro, Robotnik Automation.
- `franka_description`: Franka Robotics GmbH.
- The soft-finger package's imported manifest contained placeholder maintainer
  and license fields; its files entered this repository through HARBOT project
  history, which is preserved by Git.
