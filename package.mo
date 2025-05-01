within ;
package DynTherM "Dynamic modeling and simulation of Thermal Management systems"
  import Modelica.Math.*;
  import Modelica.Units.SI.*;
  import Modelica.Constants.*;
  import Modelica.Fluid.Utilities.*;
  import Modelica.Blocks.Interfaces.*;

  annotation (uses(Modelica(version="4.0.0"),
    ModelicaServices(version="4.0.0"),
      ExternData(version="3.0.5"),
      XogenyTest(version="1.2"),
      ExternalMedia(version="4.0.0")),
      version="2",
      conversion(noneFromVersion="1"));
end DynTherM;
