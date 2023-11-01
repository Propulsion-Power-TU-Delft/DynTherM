within DynTherM.Materials.Paints;
package BlackCoatings

  model AnodizeBlack
    extends BasePaint(
      abs=0.88,
      eps=0.88);
  end AnodizeBlack;

  model CarbonBlackPaint
    extends BasePaint(
      abs=0.96,
      eps=0.88);
  end CarbonBlackPaint;

  model CatalacBlackPaint
    extends BasePaint(
      abs=0.96,
      eps=0.88);
  end CatalacBlackPaint;

  model ChemGlazeBlackPaint
    extends BasePaint(
      abs=0.96,
      eps=0.91);
  end ChemGlazeBlackPaint;

  model DelrinBlackPlastic
    extends BasePaint(
      abs=0.96,
      eps=0.87);
  end DelrinBlackPlastic;

  model PaladinBlackLacquer
    extends BasePaint(
      abs=0.95,
      eps=0.75);
  end PaladinBlackLacquer;

  model ParsonBlackPaint
    extends BasePaint(
      abs=0.98,
      eps=0.91);
  end ParsonBlackPaint;

  model PolyethyleneBlackPlastic
    extends BasePaint(
      abs=0.93,
      eps=0.92);
  end PolyethyleneBlackPlastic;

  model TedlarBlackPlastic
    extends BasePaint(
      abs=0.94,
      eps=0.90);
  end TedlarBlackPlastic;

  model VelestatBlackPlastic
    extends BasePaint(
      abs=0.96,
      eps=0.85);
  end VelestatBlackPlastic;
end BlackCoatings;
