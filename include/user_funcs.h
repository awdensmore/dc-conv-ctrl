
/* Private typedef ---------------------------------------------------------*/
#define  FREQ_CLK   (uint32_t)(8000000)
#define  FREQ_EIS   (uint32_t)(1000)
#define  FREQ_PWM   (uint32_t)(20000)
#define  PRD_PWM    (uint32_t)(6 * (FREQ_CLK / FREQ_PWM) - 1)
#define  DC_PWM     (uint32_t)(35)

/* Private function prototypes -----------------------------------------------*/
void TIM_PWM_Adjust(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *configOC, uint32_t pulse_prd, uint32_t channel);

static const uint16_t sine_lookup[30] =
{
  500, 603, 703, 793, 871, 933, 975, 997, 997, 975, 933, 871, 793, 703, 603, \
  500, 396, 296, 206, 128, 66, 24, 2, 2, 24, 66, 128, 206, 296, 396
};


