# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# `main` 会运行 PEP 257 文档字符串规范检查。
from ament_pep257.main import main
# `pytest` 用于加测试标签。
import pytest


# 标记这是 linter 测试。
@pytest.mark.linter
# 标记这是 PEP 257 文档字符串检查。
@pytest.mark.pep257
def test_pep257():
    # 对当前目录和测试目录执行文档字符串规范检查。
    rc = main(argv=['.', 'test'])
    # 要求检查结果成功通过。
    assert rc == 0, 'Found code style errors / warnings'
