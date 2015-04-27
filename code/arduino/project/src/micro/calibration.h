#ifndef _CALIBRATION_
#define _CALIBRATION_

#define WINDOW_WIDTH  8
#define NBEACONS 4
#define NPIXELS 102
#define NCAMS 6

int angle2px[360] = 
{
  0,
  2,
  3,
  5,
  7,
  9,
  10,
  12,
  14,
  15,
  17,
  19,
  20,
  22,
  24,
  26,
  27,
  29,
  31,
  32,
  34,
  36,
  38,
  39,
  41,
  43,
  44,
  46,
  48,
  49,
  51,
  53,
  55,
  56,
  58,
  60,
  61,
  63,
  65,
  66,
  68,
  70,
  72,
  73,
  75,
  77,
  78,
  80,
  82,
  84,
  85,
  87,
  89,
  90,
  92,
  94,
  95,
  97,
  99,
  101,
  102,
  104,
  106,
  107,
  109,
  111,
  113,
  114,
  116,
  118,
  119,
  121,
  123,
  124,
  126,
  128,
  130,
  131,
  133,
  135,
  136,
  138,
  140,
  141,
  143,
  145,
  147,
  148,
  150,
  152,
  153,
  155,
  157,
  159,
  160,
  162,
  164,
  165,
  167,
  169,
  170,
  172,
  174,
  176,
  177,
  179,
  181,
  182,
  184,
  186,
  188,
  189,
  191,
  193,
  194,
  196,
  198,
  199,
  201,
  203,
  205,
  206,
  208,
  210,
  211,
  213,
  215,
  217,
  218,
  220,
  222,
  223,
  225,
  227,
  228,
  230,
  232,
  234,
  235,
  237,
  239,
  240,
  242,
  244,
  245,
  247,
  249,
  251,
  252,
  254,
  256,
  257,
  259,
  261,
  263,
  264,
  266,
  268,
  269,
  271,
  273,
  274,
  276,
  278,
  280,
  281,
  283,
  285,
  286,
  288,
  290,
  292,
  293,
  295,
  297,
  298,
  300,
  302,
  303,
  305,
  307,
  309,
  310,
  312,
  314,
  315,
  317,
  319,
  320,
  322,
  324,
  326,
  327,
  329,
  331,
  332,
  334,
  336,
  338,
  339,
  341,
  343,
  344,
  346,
  348,
  349,
  351,
  353,
  355,
  356,
  358,
  360,
  361,
  363,
  365,
  367,
  368,
  370,
  372,
  373,
  375,
  377,
  378,
  380,
  382,
  384,
  385,
  387,
  389,
  390,
  392,
  394,
  395,
  397,
  399,
  401,
  402,
  404,
  406,
  407,
  409,
  411,
  413,
  414,
  416,
  418,
  419,
  421,
  423,
  424,
  426,
  428,
  430,
  431,
  433,
  435,
  436,
  438,
  440,
  442,
  443,
  445,
  447,
  448,
  450,
  452,
  453,
  455,
  457,
  459,
  460,
  462,
  464,
  465,
  467,
  469,
  471,
  472,
  474,
  476,
  477,
  479,
  481,
  482,
  484,
  486,
  488,
  489,
  491,
  493,
  494,
  496,
  498,
  499,
  501,
  503,
  505,
  506,
  508,
  510,
  511,
  513,
  515,
  517,
  518,
  520,
  522,
  523,
  525,
  527,
  528,
  530,
  532,
  534,
  535,
  537,
  539,
  540,
  542,
  544,
  546,
  547,
  549,
  551,
  552,
  554,
  556,
  557,
  559,
  561,
  563,
  564,
  566,
  568,
  569,
  571,
  573,
  574,
  576,
  578,
  580,
  581,
  583,
  585,
  586,
  588,
  590,
  592,
  593,
  595,
  597,
  598,
  600,
  602,
  603,
  605,
  607,
  609,
  610,
  611};


  #endif

/*
   int px2angle[10]=//NPIXELS*NCAMS] =
   {0,
   6,
   12,
   18,
   24,
   29,
   35,
   41,
   47,
   53,
   59,
   65,
   71,
   77,
   82,
   88,
   94,
   100,
   106,
   112,
   118,
   124,
   130,
   136,
   141,
   147,
   153,
   159,
   165,
   171,
   177,
   183,
   189,
   194,
   200,
   206,
   212,
   218,
   224,
   230,
   236,
   242,
   247,
   253,
   259,
   265,
   271,
   277,
   283,
   289,
   295,
   300,
   306,
   312,
   318,
   324,
   330,
   336,
   342,
   348,
   354,
   359,
   365,
   371,
   377,
   383,
   389,
   395,
   401,
   407,
   412,
   418,
   424,
   430,
   436,
   442,
   448,
   454,
   460,
465,
  471,
  477,
  483,
  489,
  495,
  501,
  507,
  513,
  518,
  524,
  530,
  536,
  542,
  548,
  554,
  560,
  566,
  572,
  577,
  583,
  589,
  595,
  601,
  607,
  613,
  619,
  625,
  630,
  636,
  642,
  648,
  654,
  660,
  666,
  672,
  678,
  683,
  689,
  695,
  701,
  707,
  713,
  719,
  725,
  731,
  736,
  742,
  748,
  754,
  760,
  766,
  772,
  778,
  784,
  790,
  795,
  801,
  807,
  813,
  819,
  825,
  831,
  837,
  843,
  848,
  854,
  860,
  866,
  872,
  878,
  884,
  890,
  896,
  901,
  907,
  913,
  919,
  925,
  931,
  937,
  943,
  949,
  955,
  960,
  966,
  972,
  978,
  984,
  990,
  996,
  1002,
  1008,
  1013,
  1019,
  1025,
  1031,
  1037,
  1043,
  1049,
  1055,
  1061,
  1066,
  1072,
  1078,
  1084,
  1090,
  1096,
  1102,
  1108,
  1114,
  1119,
  1125,
  1131,
  1137,
  1143,
  1149,
  1155,
  1161,
  1167,
  1173,
  1178,
  1184,
  1190,
  1196,
  1202,
  1208,
  1214,
  1220,
  1226,
  1231,
  1237,
  1243,
  1249,
  1255,
  1261,
  1267,
  1273,
  1279,
  1284,
  1290,
  1296,
  1302,
  1308,
  1314,
  1320,
  1326,
  1332,
  1337,
  1343,
  1349,
  1355,
  1361,
  1367,
  1373,
  1379,
  1385,
  1391,
  1396,
  1402,
  1408,
  1414,
  1420,
  1426,
  1432,
  1438,
  1444,
  1449,
  1455,
  1461,
  1467,
  1473,
  1479,
  1485,
  1491,
  1497,
  1502,
  1508,
  1514,
  1520,
  1526,
  1532,
  1538,
  1544,
  1550,
  1555,
  1561,
  1567,
  1573,
  1579,
  1585,
  1591,
  1597,
  1603,
  1609,
  1614,
  1620,
  1626,
  1632,
  1638,
  1644,
  1650,
  1656,
  1662,
  1667,
  1673,
  1679,
  1685,
  1691,
  1697,
  1703,
  1709,
  1715,
  1720,
  1726,
  1732,
  1738,
  1744,
  1750,
  1756,
  1762,
  1768,
  1773,
  1779,
  1785,
  1791,
  1797,
  1803,
  1809,
  1815,
  1821,
  1827,
  1832,
  1838,
  1844,
  1850,
  1856,
  1862,
  1868,
  1874,
  1880,
  1885,
  1891,
  1897,
  1903,
  1909,
  1915,
  1921,
  1927,
  1933,
  1938,
  1944,
  1950,
  1956,
  1962,
  1968,
  1974,
  1980,
  1986,
  1991,
  1997,
  2003,
  2009,
  2015,
  2021,
  2027,
  2033,
  2039,
  2045,
  2050,
  2056,
  2062,
  2068,
  2074,
  2080,
  2086,
  2092,
  2098,
  2103,
  2109,
  2115,
  2121,
  2127,
  2133,
  2139,
  2145,
  2151,
  2156,
  2162,
  2168,
  2174,
  2180,
  2186,
  2192,
  2198,
  2204,
  2209,
  2215,
  2221,
  2227,
  2233,
  2239,
  2245,
  2251,
  2257,
  2263,
  2268,
  2274,
  2280,
  2286,
  2292,
  2298,
  2304,
  2310,
  2316,
  2321,
  2327,
  2333,
  2339,
  2345,
  2351,
  2357,
  2363,
  2369,
  2374,
  2380,
  2386,
  2392,
  2398,
  2404,
  2410,
  2416,
  2422,
  2427,
  2433,
  2439,
  2445,
  2451,
  2457,
  2463,
  2469,
  2475,
  2481,
  2486,
  2492,
  2498,
  2504,
  2510,
  2516,
  2522,
  2528,
  2534,
  2539,
  2545,
  2551,
  2557,
  2563,
  2569,
  2575,
  2581,
  2587,
  2592,
  2598,
  2604,
  2610,
  2616,
  2622,
  2628,
  2634,
  2640,
  2645,
  2651,
  2657,
  2663,
  2669,
  2675,
  2681,
  2687,
  2693,
  2699,
  2704,
  2710,
  2716,
  2722,
  2728,
  2734,
  2740,
  2746,
  2752,
  2757,
  2763,
  2769,
  2775,
  2781,
  2787,
  2793,
  2799,
  2805,
  2810,
  2816,
  2822,
  2828,
  2834,
  2840,
  2846,
  2852,
  2858,
  2864,
  2869,
  2875,
  2881,
  2887,
  2893,
  2899,
  2905,
  2911,
  2917,
  2922,
  2928,
  2934,
  2940,
  2946,
  2952,
  2958,
  2964,
  2970,
  2975,
  2981,
  2987,
  2993,
  2999,
  3005,
  3011,
  3017,
  3023,
  3028,
  3034,
  3040,
  3046,
  3052,
  3058,
  3064,
  3070,
  3076,
  3082,
  3087,
  3093,
  3099,
  3105,
  3111,
  3117,
  3123,
  3129,
  3135,
  3140,
  3146,
  3152,
  3158,
  3164,
  3170,
  3176,
  3182,
  3188,
  3193,
  3199,
  3205,
  3211,
  3217,
  3223,
  3229,
  3235,
  3241,
  3246,
  3252,
  3258,
  3264,
  3270,
  3276,
  3282,
  3288,
  3294,
  3300,
  3305,
  3311,
  3317,
  3323,
  3329,
  3335,
  3341,
  3347,
  3353,
  3358,
  3364,
  3370,
  3376,
  3382,
  3388,
  3394,
  3400,
  3406,
  3411,
  3417,
  3423,
  3429,
  3435,
  3441,
  3447,
  3453,
  3459,
  3464,
  3470,
  3476,
  3482,
  3488,
  3494,
  3500,
  3506,
  3512,
  3518,
  3523,
  3529,
  3535,
  3541,
  3547,
  3553,
  3559,
  3565,
  3571,
  3576,
  3582,
  3588,
  3594,
  3600};*/
