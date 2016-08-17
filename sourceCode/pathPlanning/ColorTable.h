#ifndef COLORTABLE_H
#define COLORTABLE_H

class ColorTable {
public:
  enum COLOR_TYPE {WHITE = 0, BLACK, RED, GREEN, BLUE, COLOR_COUNT};

  static float* GetColor(COLOR_TYPE color) {
    static const float color_table[COLOR_COUNT][3] = {
      {1.0, 1.0, 1.0},
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0}};

    return (float *)(&color_table[color]);
  }
};

#endif // COLORTABLE_H
