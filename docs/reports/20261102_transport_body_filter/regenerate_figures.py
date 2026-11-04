#!/usr/bin/env python3
"""Regenerate report plots from the bundled evidence; no ROS connections."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT = Path(__file__).resolve().parent
E = ROOT / 'evidence'
F = ROOT / 'figures'
font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if font.exists():
    font_manager.fontManager.addfont(str(font))
    plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
plt.rcParams.update({'font.size': 12, 'axes.unicode_minus': False,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'svg.fonttype': 'path'})
BLUE, ORANGE, GREEN = '#28699a', '#ce6d2d', '#237e66'


def read(name):
    return json.loads((E / name).read_text())


def finish(fig, name, note, bottom=.12):
    fig.text(.5, .025, note, ha='center', va='bottom', fontsize=10, color='#444444')
    fig.tight_layout(rect=(0, bottom, 1, .94))
    fig.savefig(F / (name + '.png'), dpi=165, facecolor='white')
    fig.savefig(F / (name + '.svg'), facecolor='white')
    plt.close(fig)


def label_bars(ax, bars, fmt='.1f'):
    ax.bar_label(bars, labels=[format(b.get_height(), fmt) for b in bars], padding=4)
    ax.margins(y=.24)
    ax.grid(axis='y', alpha=.18)
    ax.set_axisbelow(True)


def main():
    F.mkdir(exist_ok=True)
    d = read('recurrence.json')
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.6))
    fig.suptitle('1. 드라이버는 200 Hz인데, FAST-LIO가 받는 IMU는 약 25 Hz였음', fontsize=17)
    x = np.arange(2)
    axes[0].set_xticks(x, ['느린 상태', '같은 명령으로 재시작'])
    for offset, key, title, color in [(-.18, 'driver_hz', '드라이버 발행', BLUE),
                                     (.18, 'imu_hz', 'FAST-LIO 수신', ORANGE)]:
        bars = axes[0].bar(x+offset, [d[k][key] for k in ['bad','restarted_goal']], .36, label=title, color=color)
        label_bars(axes[0], bars)
    axes[0].set_ylabel('IMU 빈도 [Hz]'); axes[0].legend(loc='upper center', ncol=2, fontsize=10)
    bars = axes[1].bar(x, [d[k]['fusion_age_ms']['median'] for k in ['bad','restarted_goal']], color=[ORANGE, GREEN], width=.55)
    label_bars(axes[1], bars)
    axes[1].set_xticks(x, ['느린 상태', '같은 명령으로 재시작'])
    axes[1].set_ylabel('Fusion 입력 age 중앙값 [ms]')
    finish(fig,'01_recurrence','2026-10-29 저장 로그. 재시작 전후 관측이며 SHM만 바꾼 A/B 시험이 아님.\n같은 설정에서도 상태가 달라져, 버퍼 크기만으로 원인이 확정되지 않음.')

    payload = read('shm_send.json')['payload_bytes'] / 2**20
    fig, ax = plt.subplots(figsize=(11, 5.5))
    fig.suptitle('2. 작은 기본 SHM에서, 발행 데이터 크기에 맞춘 participant별 설정으로', fontsize=16)
    bars = ax.bar(['문서상 기본값\n0.5 MiB', '일반 / Livox\n32 MiB', '큰 PCD 발행\n128 MiB'], [.5,32,128], color=[ORANGE,BLUE,GREEN], width=.5)
    label_bars(ax,bars)
    ax.axhline(payload,color='#9f3b68',linestyle='--',label=f'/cloud_pcd 1회 직렬화 크기: {payload:.2f} MiB')
    ax.set_ylabel('용량 [MiB]');ax.legend(loc='upper left')
    finish(fig,'02_shm_capacity','일반 구독자가 128 MiB를 별도로 복사 할당해야 한다는 뜻은 아님.\n128 MiB 송신 participant → 32 MiB 수신 participant에서 약 34.53 MiB 메시지 3/3 수신 확인.')

    a,b=read('udp_before_passive.json'),read('udp_after_passive.json')
    fig, axes = plt.subplots(1,2,figsize=(12,5.5))
    fig.suptitle('3. 커널 상한뿐 아니라 DDS 소켓 요청 크기를 바꾼 뒤 수신 오류 감소',fontsize=16)
    for ax,key,title in zip(axes,['RcvbufErrors','SndbufErrors'],['수신 버퍼 오류','송신 버퍼 오류 — 남은 문제']):
        bars=ax.bar(['명시 요청 전','4 MiB 요청 + 재시작 후'],[a['udp_delta'][key],b['udp_delta'][key]],color=[ORANGE,GREEN],width=.55)
        label_bars(ax,bars,'.0f');ax.set_title(title);ax.set_ylabel('각 10초간 시스템 UDP 카운터 증가 [건]')
    finish(fig,'03_udp_errors','전후 모두 커널 상한 16 MiB / 확대 SHM 사용. 추가 ROS 측정 노드 없이 각각 10초 관측.\n서로 다른 시간대의 관측 비교. 오류 카운터는 Livox 센서의 패킷 손실 수와 같지 않음.')

    d=read('height_geometry_summary.json')['bags']
    fig,axes=plt.subplots(1,2,figsize=(13,5.8))
    fig.suptitle('4. 문제 구간 180–240초: 바닥점 통과 억제와 몸체 높이 후보 보존',fontsize=16)
    x=np.arange(2);labels=['20:09:54 bag','20:50:26 bag']
    for shift,key,title,color in [(-.24,'old_local','기존 절대 Z',ORANGE),(0,'abs005_local','절대 Z 하한 −0.05 m',BLUE),(.24,'new','BODY / 중력 기준',GREEN)]:
        ws=[next(w for w in d[k]['windows'] if w['window']=='problem') for k in ['200954','205026']]
        for ax,metric in zip(axes,['floor_retained_pct','body_band_candidate_retained_pct']):
            vals=[w[key][metric] for w in ws]
            bars=ax.bar(x+shift,vals,.24,color=color,label=title)
            ax.bar_label(bars,labels=[f'{v:.3f}' if 0<v<.01 else f'{v:.2f}' for v in vals],padding=3,fontsize=10)
    for ax in axes:
        ax.set_xticks(x,labels);ax.set_ylim(0,114);ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True)
    axes[0].set_ylabel('바닥으로 추정한 점 중 통과 비율 [%]');axes[0].set_title('낮을수록 바닥 오검출 감소')
    axes[1].set_ylabel('몸체 높이 후보점 중 통과 비율 [%]');axes[1].set_title('독립 기하 기준의 후보점 보존')
    fig.legend(*axes[0].get_legend_handles_labels(),loc='lower center',bbox_to_anchor=(.5,.11),ncol=3,fontsize=10)
    finish(fig,'04_height_points','RANSAC 바닥 / 몸체 높이 후보 분류를 사용한 점 단위 지표. 사람·상자 검출 정답 라벨에 대한 정확도가 아님.\n절대 Z 하한만 −0.05 m로 바꿔도 이 표의 바닥점 대부분은 제거됨. BODY 방식의 핵심은 높이 기준의 일관성.',bottom=.22)

    d=read('height_costmap_summary.json')
    fig, axes = plt.subplots(1,2,figsize=(13,5.6))
    fig.suptitle('5. 실제 Nav2 costmap 재계산: 길목의 lethal cell 감소',fontsize=16)
    vals=[d['costmaps'][k]['lethal_p50_p95'][0] for k in ['old_local','abs005_local','new_local']]
    bars=axes[0].bar(['기존 절대 Z','절대 Z −0.05 m','BODY / 중력'],vals,color=[ORANGE,BLUE,GREEN]);label_bars(axes[0],bars)
    axes[0].set_title('Local: 6 m × 6 m, lethal cell 중앙값');axes[0].set_ylabel('셀 수')
    vals=[d['global_additional_lethal'][k]['additional_lethal_on_static_free_p50_p95'][0] for k in ['old_global','new_global']]
    bars=axes[1].bar(['기존','BODY / 중력'],vals,color=[ORANGE,GREEN],width=.5);label_bars(axes[1],bars,'.0f')
    axes[1].set_title('Global: 정적 지도 free 셀에 추가된 lethal');axes[1].set_ylabel('로봇 주변 8 m 영역의 셀 수 중앙값')
    finish(fig,'05_costmap_cells','20:09:54 bag 기반, 관측 전달 시각을 통제한 Nav2 비교. Global 세부 집계 180–235초.\n셀 감소는 정확도 개선율이 아님. 낮은 실제 물체도 필터의 의도에 따라 제외됨. 실제 주행 명령은 실행하지 않음.')


if __name__=='__main__':
    main()
