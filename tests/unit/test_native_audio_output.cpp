#include "ps_audio/native_output.h"
#include "cycle_pipe_fixtures.h"
#include "cycle_fixtures.h"
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <cmath>
using namespace ps;
using namespace ps_audio;
namespace {
class FixtureSource final:public OutputSource {
public:
    std::atomic<bool> available{true};
    std::atomic<std::size_t> produced{0};
    std::size_t read(float* out,std::size_t n) override {
        if(!available) return 0;
        std::fill(out,out+n*2,.25f); produced.fetch_add(n); return n;
    }
    int channels() const override { return 2; }
    int sample_rate() const override { return 48000; }
    bool connected() const override { return true; }
};
bool wait_for_frames(const OutputPump& pump,std::size_t frames) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(pump.stats().queued_frames<frames && std::chrono::steady_clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return pump.stats().queued_frames>=frames;
}
}
TEST_CASE("Native audio reader lease rejects a second consumer and survives invalidation", "[native_output]") {
    VoiceSettings settings; settings.engine.dx_intake_m=.1; settings.engine.dx_exhaust_m=.1;
    Voice voice(test::make_pipe_i4_cycle_desc(),test::kCycleFixtureLhv,test::kCycleFixtureAfr,settings);
    Runner runner({&voice}); auto bridge=std::make_shared<RunnerOutput>(runner,48000);
    auto source=bridge->acquire(); REQUIRE(source); CHECK_FALSE(bridge->acquire());
    REQUIRE(runner.render_block()); std::vector<float> buf(256*static_cast<std::size_t>(runner.channel_count()));
    CHECK(runner.read(buf.data(),256)==0); CHECK(runner.rejected_reads()==1);
    CHECK(source->read(buf.data(),256)==256); CHECK(source->synthesis_stats().render_ns>0);
    source.reset(); REQUIRE(runner.render_block()); CHECK(runner.read(buf.data(),256)==256);
    source=bridge->acquire(); REQUIRE(source); bridge->invalidate();
    CHECK_FALSE(source->connected()); CHECK(source->read(buf.data(),256)==0); source.reset();
}
TEST_CASE("Native output pump feeds a separate device consumer while render thread is idle", "[native_output]") {
    auto source=std::make_shared<FixtureSource>(); OutputPump pump(source,12,.5f); pump.start();
    REQUIRE(wait_for_frames(pump,576)); std::atomic<std::size_t> delivered{0}; std::atomic<bool> valid{true};
    std::thread device([&] {
        std::vector<float> buf(960);
        for(int i=0;i<20;++i) {
            if(!wait_for_frames(pump,480)) { valid=false; break; }
            const auto got=pump.read(buf.data(),480); delivered.fetch_add(got);
            for(std::size_t j=0;j<got*2;++j) if(!std::isfinite(buf[j]) || buf[j]<0 || buf[j]>.125f) valid=false;
        }
    });
    // No _process feed, positions or queue pushes for a 126 ms render hitch.
    std::this_thread::sleep_for(std::chrono::milliseconds(126)); device.join(); pump.stop();
    CHECK(valid.load()); CHECK(delivered.load()==9600);
    CHECK(pump.stats().output_missing_frames==0); CHECK(pump.stats().source_missing_frames==0);
    CHECK(pump.stats().queued_frames<=2400); // hard 50 ms queue bound
}
TEST_CASE("Native output separates synthesis starvation from device short reads", "[native_output]") {
    auto source=std::make_shared<FixtureSource>(); source->available=false;
    OutputPump pump(source,5); pump.start(); std::this_thread::sleep_for(std::chrono::milliseconds(10)); pump.stop();
    std::vector<float> out(480); CHECK(pump.read(out.data(),240)==0);
    CHECK(pump.stats().source_short_reads>0); CHECK(pump.stats().source_missing_frames>0);
    CHECK(pump.stats().output_short_reads==1); CHECK(pump.stats().output_missing_frames==240);
    CHECK_THROWS_AS(OutputPump(source,100),std::invalid_argument);
}

TEST_CASE("Native output survives destruction of its voice owner and returns reader ownership on detach", "[native_output]") {
    VoiceSettings settings; settings.engine.dx_intake_m=.1; settings.engine.dx_exhaust_m=.1;
    auto voice=std::make_unique<Voice>(test::make_pipe_i4_cycle_desc(),test::kCycleFixtureLhv,test::kCycleFixtureAfr,settings);
    auto runner=std::make_unique<Runner>(std::vector<Voice*>{voice.get()});
    auto bridge=std::make_shared<RunnerOutput>(*runner,48000);
    const auto channels=static_cast<std::size_t>(runner->channel_count());
    runner->start();
    {
        OutputPump pump(bridge->acquire(),12); pump.start();
        REQUIRE(wait_for_frames(pump,32));
        bridge->invalidate(); runner->stop(); runner.reset(); voice.reset();
        CHECK_FALSE(pump.source_connected());
        std::vector<float> buffer(128*channels);
        (void)pump.read(buffer.data(),128); pump.stop();
    }
    CHECK_FALSE(bridge->acquire());
}
