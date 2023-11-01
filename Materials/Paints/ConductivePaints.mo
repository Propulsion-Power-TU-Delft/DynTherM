within DynTherM.Materials.Paints;
package ConductivePaints
  model BrilliantAluminumPaint
    extends BasePaint(
      abs=0.30,
      eps=0.31);
  end BrilliantAluminumPaint;

  model ChromericSilverPaint
    extends BasePaint(
      abs=0.30,
      eps=0.30);
  end ChromericSilverPaint;

  model DupontSilverPaint
    extends BasePaint(
      abs=0.43,
      eps=0.49);
  end DupontSilverPaint;

  model EpoxyAluminumPaint
    extends BasePaint(
      abs=0.77,
      eps=0.81);
  end EpoxyAluminumPaint;

  model SiliconeAluminumPaint
    extends BasePaint(
      abs=0.29,
      eps=0.30);
  end SiliconeAluminumPaint;
end ConductivePaints;
