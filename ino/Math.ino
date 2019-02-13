// ====================================================
// =============== HELPER FUNCTIONS ==================
// ================= math  ========================
// =====================================================

double map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double HP_filter(double x, double xmin) {
  if (abs(x) > xmin) return x;
  return 0;
}

bool HP_filter_stick(double x1, double x2, double rmin) {
  x1 = abs(x1);
  x2 = abs(x2);
  if (sqrt(sq(x1)+sq(x2)) > rmin) return 1;
  return 0;
}
