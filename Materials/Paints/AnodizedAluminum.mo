within DynTherM.Materials.Paints;
package AnodizedAluminum

  model Black
    extends BasePaint(
      abs=0.65,
      eps=0.82);
  end Black;

  model Blue
    extends BasePaint(
      abs=0.53,
      eps=0.82);
  end Blue;

  model Brown
    extends BasePaint(
      abs=0.73,
      eps=0.86);
  end Brown;

  model Chromic
    extends BasePaint(
      abs=0.44,
      eps=0.56);
  end Chromic;

  model Clear
    extends BasePaint(
      abs=0.27,
      eps=0.76);
  end Clear;

  model Gold
    extends BasePaint(
      abs=0.48,
      eps=0.82);
  end Gold;

  model Green
    extends BasePaint(
      abs=0.66,
      eps=0.88);
  end Green;

  model Plain
    extends BasePaint(
      abs=0.26,
      eps=0.04);
  end Plain;

  model Red
    extends BasePaint(
      abs=0.57,
      eps=0.88);
  end Red;

  model Yellow
    extends BasePaint(
      abs=0.47,
      eps=0.87);
  end Yellow;
end AnodizedAluminum;
