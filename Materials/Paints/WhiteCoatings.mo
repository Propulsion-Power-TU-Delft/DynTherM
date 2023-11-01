within DynTherM.Materials.Paints;
package WhiteCoatings
  model BiphenylWhiteSolid
    extends BasePaint(
      abs=0.23,
      eps=0.86);
  end BiphenylWhiteSolid;

  model CatalacWhitePaint
    extends BasePaint(
      abs=0.24,
      eps=0.90);
  end CatalacWhitePaint;

  model DupontLuciteAcrylicLacquer
    extends BasePaint(
      abs=0.35,
      eps=0.90);
  end DupontLuciteAcrylicLacquer;

  model OpalGlass
    extends BasePaint(
      abs=0.28,
      eps=0.87);
  end OpalGlass;

  model SperexWhitePaint
    extends BasePaint(
      abs=0.34,
      eps=0.85);
  end SperexWhitePaint;

  model TedlarWhitePlastic
    extends BasePaint(
      abs=0.39,
      eps=0.87);
  end TedlarWhitePlastic;

  model ZincOxideSodiumSilicate
    extends BasePaint(
      abs=0.15,
      eps=0.92);
  end ZincOxideSodiumSilicate;
end WhiteCoatings;
