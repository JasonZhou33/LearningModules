#define PC 0
#define MANIFOLD 1

// The manifold(from dji, based on Jetson TK1) is ARM architecture.
// Most of the PC is x86 architecture.
#if defined __arm__
#define PLATFORM MANIFOLD
#else
#define PLATFORM PC
#endif